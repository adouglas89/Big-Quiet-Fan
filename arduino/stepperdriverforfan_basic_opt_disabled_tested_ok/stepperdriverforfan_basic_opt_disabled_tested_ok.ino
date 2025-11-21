#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>

// ---------- USER PINS (Raspberry Pi Pico 2W) ----------
static const int PIN_PWM_AH = 4;
static const int PIN_PWM_AL = 5;
static const int PIN_PWM_BH = 6;
static const int PIN_PWM_BL = 7;
static const int En_a       = 8;
static const int En_b       = 9;
static const int POT_PIN    = 26;  // ADC0
static const int CURR_PIN   = 28;  // ADC2

// ---------- MOTOR / DRIVER ----------
StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(PIN_PWM_AH, PIN_PWM_AL, PIN_PWM_BH, PIN_PWM_BL, En_a, En_b);

float anticog_multiplier = 25.0f;//this should be the number of pole pairs but a float
float voltage_override = 1.0f;

float maybe_o = 0.0f;
const int lut_size = 200; // Adjust size as needed
float lut[lut_size]; // Lookup table
int indexa = 0;
// ---------- SPEED / POT ----------
volatile float RPS       = 0.0f;
volatile float RPS_accel = 0.3f;    // rps^2 
volatile float rps_max   = 8.33f;
const float    HYST_RPS  = 0.1f;    // rps hysteresis band
unsigned long  lastMicros = 0;
float v_drive_befoverlay = 3.0f;
float cmd_rad_per_sec = 0.0f;
// ---------- CURRENT READ ----------
float read_current_units() {
  // raw ADC voltage (0..3.3 V) proportional to current; absolute amps not required for minimization
  int raw = analogRead(CURR_PIN);
  return 3.3f * (float)raw / 4095.0f;
}
void stall_detect(){
 if (read_current_units() > 1.4){
   voltage_override = 0.0f;
 }

}
void receiveLUT() {
    int index2 = 0;
    int counter = 0;
    Serial.println("ready for lut");

    while (index2 < lut_size) {
        if (Serial.available() >= 5) { // 4 bytes float + 1 bytes checksum +1 end byte
            byte buffer[4];
            for (int i = 0; i < 4; i++) {
                buffer[i] = Serial.read();
            }

            byte received_checksum = Serial.read(); 

            // Calculate checksum
            byte calculated_checksum = 0;
            for (int i = 0; i < 4; i++) {
                calculated_checksum ^= buffer[i];
            }

            if (calculated_checksum == received_checksum) {
                // Convert bytes to float
                float received_value;
                unsigned char *valueBytes = (unsigned char*)&received_value;
                for (int i = 0; i < 4; i++) {
                    valueBytes[i] = buffer[i];
                }

                lut[index2] = received_value;

                // Send acknowledgment
                Serial.println("A");


                index2++;
            } else {
                Serial.println("N");
            }
        }
motor.move(cmd_rad_per_sec); //gotta keep motor moving
    
    }

    //Serial.println("LUT Received");
}
void initialize_lut() {
    for (int i = 0; i < lut_size; i++) {
        lut[i] = 0.0f;
    }
}

void SerialComm() {
    if (Serial.available() > 0) {
        switch (Serial.peek()) {
            case 's': Serial.read(); Serial.print("s"); Serial.println(motor.shaftVelocity()); break;
            case 'b': Serial.read(); Serial.print("b"); Serial.println(anticog_multiplier); break;
            case 'U': Serial.read(); anticog_multiplier = Serial.parseFloat(); break;
            case 'L': Serial.read();  receiveLUT(); break;
            case 'O':
                Serial.read();
                maybe_o = Serial.parseFloat();
                voltage_override = (maybe_o >= 0.999) ? 1.0f : 0.0f;
                break;
            default: Serial.read(); break;
        }
    }
}

float anti_cog_voltage() {
    float anticog_angle = fmod(motor.shaft_angle * motor.pole_pairs, 2.0f * PI);
    indexa = int(((anticog_angle / (2.0f * PI)) * float(lut_size-1)));  
    return lut[indexa];
}
// ---------- OPTIMIZER (ΔV trim around base voltage) ----------
struct MinCurrentTrimOpt {
  // Voltage bounds (motor command)
  float Vmin = 3.0f;
  float Vmax = 24.0f;

  // Trim state (what we adjust): Vcmd = clamp(Vbase + trim)
  float trim = 0.0f;

  // Step logic
  float step     = 0.10f;   // initial trim step (V)
  float step_max = 0.30f;
  float step_min = 0.02f;
  int   dir      = +1;      // +1 / -1

  // Filtering & decision
  float Ibar   = 0.0f;      // EMA of current
  float alpha  = 0.15f;     // EMA coeff (0..1)
  float Iref   = INFINITY;  // drifting reference (best so far with drift)
  float eps    = 0.02f;     // significant improvement threshold 
  float drift  = 0.003f;    // upward drift rate for Iref (keeps optimizer adaptive)

  // Timing
  uint32_t lastTickUs = 0;
  uint32_t periodUs   = 10000;  // 10 ms tick
  int      settleTicks = 8;     // ~80 ms settle after each voltage change
  int      settleLeft  = 0;

  // simple rate-limited debugging
  uint16_t printEvery = 25;
  uint16_t printCnt   = 0;

  // helpers
  inline float clampV(float v) const { return constrain(v, Vmin, Vmax); }

  static inline float ema(float prev, float sample, float a) {
    return prev + a * (sample - prev);
  }

  void init() {
    trim = 0.0f;
    step = 0.10f;
    dir  = +1;
    Ibar = 0.0f;
    Iref = INFINITY;
    settleLeft  = settleTicks;
    lastTickUs  = micros();
    printCnt    = 0;
  }

  void applyCommand(float Vbase) {
    float Vcmd = clampV(Vbase + trim);
    v_drive_befoverlay = Vcmd;
  }

  void tick(float Iraw, float Vbase) {
    uint32_t now = micros();
    if (now - lastTickUs < periodUs) {
      return; // wait until next tick
    }
    lastTickUs = now;

    // Update filtered current
    Ibar = ema(Ibar, Iraw, alpha);
    if (isinf(Iref)) Iref = Ibar;            // first sample
    // Let the "best" baseline drift upward to follow changing conditions
    if (Ibar > Iref) Iref += drift * (Ibar - Iref);
    else             Iref  = Ibar;           // if we see lower current, follow it immediately

    // Settling time after a voltage change
    if (settleLeft > 0) {
      settleLeft--;
      applyCommand(Vbase);
      return;
    }

    bool improved = (Ibar + eps < Iref);

    if (improved) {
      // Accept direction; slightly grow step
      step = min(step * 1.2f, step_max);
      // Advance trim in same direction
      trim += dir * step;
      // Keep command within bounds (relative to Vbase)
      float Vcmd = clampV(Vbase + trim);
      // If clamped, flip and shrink
      if (Vcmd == Vmin || Vcmd == Vmax) {
        trim = Vcmd - Vbase;
        dir  = -dir;
        step = max(step * 0.5f, step_min);
      }
      settleLeft = settleTicks;
      Iref = Ibar; // new local best
      applyCommand(Vbase);
    } else {
      // Backtrack: flip direction & shrink step
      dir  = -dir;
      step = max(step * 0.5f, step_min);
      trim += dir * step;
      // Bound again
      float Vcmd = clampV(Vbase + trim);
      if (Vcmd == Vmin || Vcmd == Vmax) {
        trim = Vcmd - Vbase;
      }
      settleLeft = settleTicks;
      applyCommand(Vbase);
      //Serial.println(motor.voltage_limit);
    }

    // (Optional) light debug printing, rate-limited
   // if (++printCnt >= printEvery) {
   //   printCnt = 0;
  //    Serial.print("Ib"); Serial.print(Ibar, 3);
  //    Serial.print(" I"); Serial.print(Iref, 3);
 //     Serial.print(" V"); Serial.print(Vbase, 3);
 //     Serial.print(" t");  Serial.print(trim, 3);
 //     Serial.print(" s");  Serial.print(step, 3);
  //    Serial.print(" d");   Serial.println(dir);
  //  }
  }
} opt;

// ---------- SETUP ----------
void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(1);
  initialize_lut();
  // while (!Serial) {}

  // GPIO modes (SimpleFOC may reconfigure as needed)
  pinMode(PIN_PWM_AH, OUTPUT_12MA);
  pinMode(PIN_PWM_AL, OUTPUT_12MA);
  pinMode(PIN_PWM_BH, OUTPUT_12MA);
  pinMode(PIN_PWM_BL, OUTPUT_12MA);
  pinMode(LED_BUILTIN, OUTPUT);

  // ADC
  analogReadResolution(12);

  // Driver
  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit        = 24.0f;   // driver ceiling
  driver.pwm_frequency        = 30000;   // 30 kHz PWM
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    while (1) { delay(1000); }
  }
  driver.enable();

  // Motor
  motor.linkDriver(&driver);
  motor.controller         = MotionControlType::velocity_openloop;
  motor.velocity_limit     = 60.0f;           // rad/s ceiling
  motor.foc_modulation     = FOCModulationType::SinePWM;
  motor.modulation_centered = false;
  motor.init();

  opt.init();               // initialize optimizer state (after motor.init)
  lastMicros = micros();

  Serial.println("Stepper: pot-controlled RPM + online ΔV trim for min current");
}

// ---------- LOOP ----------
void loop() {
  //begin debug printing
  static unsigned long lastPrintMs = 0;
  static const unsigned long printIntervalMs = 500;
  
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs >= printIntervalMs) {
    lastPrintMs = nowMs;
    int raw = analogRead(POT_PIN);
    float norm = (float)raw / 4095.0f;
    float target = norm * rps_max;
    Serial.print("Raw pot: "); Serial.print(raw);
    Serial.print(" Target rps: "); Serial.print(target, 2);
    Serial.print(" current RPS: "); Serial.println(RPS, 2);
  }

  //end debug stuff
  //SerialComm();
  cmd_rad_per_sec = RPS * 2.0f * PI;
  int raw = analogRead(POT_PIN);
  float norm   = (float)raw / 4095.0f;      // 0..1
  float target = norm * rps_max;            // RPS target

  // --- Time step for RPS ramp ---
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  // --- RPS ramp with hysteresis & accel limit ---
  float err = target - RPS;
  if (fabsf(err) > HYST_RPS) {
    float maxStep = RPS_accel * dt;
    if (err > 0.0f) RPS += min(err,  maxStep);
    else            RPS += max(err, -maxStep);
  }
  RPS = constrain(RPS, 0.0f, rps_max);
 float Vbase = (19.0f * (RPS / rps_max)) + 4.5f;   // maps 0..rps_max -> 3..16 V
  Vbase = constrain(Vbase, opt.Vmin, opt.Vmax);     // ensure within bounds
cmd_rad_per_sec = RPS * 2.0f * PI;
  float Iraw = read_current_units();
    motor.voltage_limit = (Vbase*voltage_override);
  for (int i = 0; i<=10; i++){   

  motor.move(cmd_rad_per_sec);  // one call per loop keeps things responsive
  }
}
