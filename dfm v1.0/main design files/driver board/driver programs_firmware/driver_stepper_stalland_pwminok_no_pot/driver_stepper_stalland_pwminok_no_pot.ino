#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// ---------- USER PINS (Raspberry Pi Pico 2W) ----------
static const int PIN_PWM_AH = 4;
static const int PIN_PWM_AL = 5;
static const int PIN_PWM_BH = 6;
static const int PIN_PWM_BL = 7;
static const int En_a       = 8;
static const int En_b       = 9;
static const int POT_PIN    = 26;  // ADC0
static const int CURR_PIN   = 28;  // ADC2

// PWM throttle input (must be PWM B channel, odd GPIO > 12)
static const uint PWM_INPUT_PIN = 13;

// Duty sampler
static const uint32_t SAMPLE_US = 2000;   // 2ms sample window
static const float EMA_ALPHA = 0.15f;

static uint g_slice = 0;
static float g_duty_filt = 0.0f;

// ---------- MOTOR / DRIVER ----------
StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(PIN_PWM_AH, PIN_PWM_AL, PIN_PWM_BH, PIN_PWM_BL, En_a, En_b);

float duty = 0.0f;
float voltage_override = 1.0f;
float expected_current2 = 0.0f;

// ---------- SPEED ----------
volatile float RPS       = 0.0f;
volatile float RPS_accel = 0.3f;    // rps^2
volatile float rps_max   = 6.95f;
volatile float rps_min   = 3.0f;
const float    HYST_RPS  = 0.1f;
unsigned long  lastMicros = 0;
float cmd_rad_per_sec = 0.0f;
float curr = 0.0f;

// ---------- PWM INPUT INIT ----------
void pwm_duty_input_init(uint gpio) {
  if (pwm_gpio_to_channel(gpio) != PWM_CHAN_B) {
    while (true) {
      Serial.println("ERROR: PWM_INPUT_PIN must be a PWM B-channel GPIO (odd-numbered).");
      delay(1000);
    }
  }

  gpio_set_function(gpio, GPIO_FUNC_PWM);
  g_slice = pwm_gpio_to_slice_num(gpio);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);

  // Must match the read function
  const float clkdiv = 256.0f;
  pwm_config_set_clkdiv(&cfg, clkdiv);

  pwm_init(g_slice, &cfg, true);
  pwm_set_counter(g_slice, 0);
}

// ---------- PWM DUTY READ (NONBLOCKING + OVERFLOW SAFE) ----------
float pwm_duty_read_nonblocking() {
  static uint32_t last_sample_us = 0;
  static bool started = false;

  uint32_t now = micros();

  // On first call, just arm the timing and clear counter
  if (!started) {
    started = true;
    last_sample_us = now;
    pwm_set_counter(g_slice, 0);
    return g_duty_filt;
  }

  uint32_t dt_us = now - last_sample_us;
  if (dt_us < SAMPLE_US) {
    return g_duty_filt;
  }

  uint16_t high_counts = pwm_get_counter(g_slice);

  // Reset for next window
  pwm_set_counter(g_slice, 0);
  last_sample_us = now;

  // Convert counts -> duty
  const float f_sys = (float)clock_get_hz(clk_sys);
  const float clkdiv = 256.0f;  // must match init()
  const float count_rate = f_sys / clkdiv; // counts/sec when HIGH
  const float window_s = (float)dt_us * 1e-6f;

  const float max_counts = count_rate * window_s;
  float d = (max_counts > 0.0f) ? ((float)high_counts / max_counts) : 0.0f;

  if (d < 0.0f) d = 0.0f;
  if (d > 1.0f) d = 1.0f;

  // Optional smoothing (does NOT fix overflow; overflow is already prevented by frequent sampling)
  g_duty_filt += EMA_ALPHA * (d - g_duty_filt);
  return g_duty_filt;
}

// ---------- CURRENT READ ----------
float read_current_units() {
  int raw = analogRead(CURR_PIN);
  return 3.3f * (float)raw / 4095.0f;
}

bool stall_detect(float x, float current) {
  const float TAU_S = 0.16f;
  static float current_filt = 0.0f;
  static bool initialized = false;
  static unsigned long last_us = 0;

  unsigned long now_us = micros();
  float dt = 0.0f;

  if (last_us != 0) {
    dt = (now_us - last_us) * 1e-6f;
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 0.25f) dt = 0.25f;
  }
  last_us = now_us;

  if (!initialized) {
    current_filt = current;
    initialized = true;
  } else {
    float alpha = dt / (TAU_S + dt);
    current_filt = current_filt + alpha * (current - current_filt);
  }

  float expected_current =
      0.001f * powf(x, 4) -
      0.0141f * powf(x, 3) +
      0.1142f * powf(x, 2) -
      0.3059f * x +
      0.4257f;

  expected_current2 = expected_current;
  curr = current_filt;

  if ((expected_current * 0.6f < current_filt) &&
      (current_filt < expected_current * 1.3f + 0.15f)) {
    return false;
  } else {
    return true;
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(1);

  pinMode(PIN_PWM_AH, OUTPUT_12MA);
  pinMode(PIN_PWM_AL, OUTPUT_12MA);
  pinMode(PIN_PWM_BH, OUTPUT_12MA);
  pinMode(PIN_PWM_BL, OUTPUT_12MA);
  pinMode(LED_BUILTIN, OUTPUT);

  pwm_duty_input_init(PWM_INPUT_PIN);

  analogReadResolution(12);

  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit        = 24.0f;
  driver.pwm_frequency        = 30000;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    while (1) { delay(1000); }
  }
  driver.enable();

  motor.linkDriver(&driver);
  motor.controller           = MotionControlType::velocity_openloop;
  motor.velocity_limit       = 60.0f;
  motor.foc_modulation       = FOCModulationType::SinePWM;
  motor.modulation_centered  = false;
  motor.init();

  lastMicros = micros();
}

// ---------- LOOP ----------
void loop() {
  // IMPORTANT: update duty continuously so counter can’t overflow
  duty = pwm_duty_read_nonblocking();

  // Debug print at 500ms (printing rate no longer affects sampling!)
  static unsigned long lastPrintMs = 0;
  static const unsigned long printIntervalMs = 500;
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs >= printIntervalMs) {
    lastPrintMs = nowMs;
    //Serial.print("Duty: ");
    //Serial.println(duty, 4);
  }

  float Iraw = read_current_units();

  // Use duty as throttle (0..1)
  float norm = duty;
  float target = norm * rps_max;
  if (target < rps_min) target = rps_min;

  // Time step for RPS ramp
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  // RPS ramp
  float err = target - RPS;
  if (fabsf(err) > HYST_RPS) {
    float maxStep = RPS_accel * dt;
    if (err > 0.0f) RPS += min(err,  maxStep);
    else            RPS += max(err, -maxStep);
  }
  RPS = constrain(RPS, 0.0f, rps_max);

  float Vbase = (22.0f * (RPS / rps_max)) + 2.5f;
  Vbase = constrain(Vbase, 2.5f, 24.0f);

  cmd_rad_per_sec = RPS * 2.0f * PI;

  motor.voltage_limit = (Vbase * voltage_override);

  if (RPS > 2.8f) {
    if (stall_detect(RPS, Iraw)) {
      voltage_override = 0.0f;
    }
  }

  // Keep motor running
  motor.move(cmd_rad_per_sec);
}
