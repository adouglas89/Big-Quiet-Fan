import re
def get_end_of_file(lines):
    lines2 = []
    while True:     
        for line in reversed(lines):
            if line.startswith("G92 E0"):
                return list(reversed(lines2))
            lines2.append(line)
def get_start_of_file(lines):
    lines2 = []
    while True:
        for line in lines:
            if line.startswith("G92 E0"):
                return lines2
            lines2.append(line)
    
def remove_small_extrusion_segments(lines, output_file, extrusion_threshold, start, end):

    extern_perim = 0 
    result_lines = []
    segment_lines = []
    previous_extrusion = None
    total_extrusion = 0
    retain_segment_override = 0
    for line in lines:
        if line.startswith(";TYPE:"):
            if "External perimeter" in line:
                extern_perim = 1
            else:
                extern_perim = 0   
        if line.startswith("M104"):
            retain_segment_override = 1
        if extern_perim == 1:
            retain_segment_override = 1       
        if line.startswith("G92 E0"):
            # At the start of a new segment
            if segment_lines:
                # Process the previous segment
                if total_extrusion >= extrusion_threshold or retain_segment_override == 1:
                    result_lines.extend(segment_lines)  # Keep the segment
                    retain_segment_override = 0
                # Reset for the next segment
                segment_lines = []
                total_extrusion = 0
                previous_extrusion = None

        # Add the current line to the segment
        segment_lines.append(line)

        # Check for extrusion in G1 lines
        if not line.startswith("G92 E0"):
            match = re.search(r"E([\d.]+)", line)
            if match:
                current_extrusion = float(match.group(1))
                if previous_extrusion is not None:
                    total_extrusion += max(0,current_extrusion - previous_extrusion)
                previous_extrusion = current_extrusion
    # Handle the last segment in the file

    if segment_lines and total_extrusion >= extrusion_threshold or retain_segment_override == 1:
        result_lines.extend(segment_lines)
        retain_segment_override = 0
    
    result_lines = result_lines+end
    # Write the filtered content to the output file
    with open(output_file, 'w') as file:
        file.writelines(result_lines)
    
# Parameters
extrusion_threshold = 5  # Threshold for total extrusion length
input_file = "smallprintermotor mount 0_8mm_petg.gcode"  # Input G-code file
output_file = "f11_"+str(extrusion_threshold)+input_file  # Output G-code file

with open(input_file, 'r') as file:
    lines = file.readlines()
# Run the function
print(type(lines))

end = get_end_of_file(lines)

start = get_start_of_file(lines)    

remove_small_extrusion_segments(lines, output_file, extrusion_threshold, start, end)
