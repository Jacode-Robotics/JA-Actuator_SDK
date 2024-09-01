import re

def extract_data_from_jointld(file_path):
    extracted_data = []
    
    with open(file_path, 'r') as file:
        for line in file:
            # Match the pattern |index; followed by comma-separated float values
            match = re.match(r'\|(\d+);([\d\.\-\+,]+)', line.strip())
            if match:
                index = int(match.group(1))
                data_values = match.group(2).split(',')
                
                # Filter out any empty strings and convert the remaining data values to floats
                data_values = [float(val) for val in data_values if val]
                
                # Ensure we have exactly 12 values
                if len(data_values) == 12:
                    # Scale the positions and velocities and convert to integers
                    dxl_goal_position = [int((val / 360.0) * 32768) for val in data_values[:6]]
                    dxl_goal_velocity = [int((val / 360.0) * 32768) for val in data_values[6:12]]
                    
                    extracted_data.append({
                        'index': index,
                        'dxl_goal_position': dxl_goal_position,
                        'dxl_goal_velocity': dxl_goal_velocity
                    })
    
    return extracted_data

# Example usage
file_path = 'JointLD_25.data'
data = extract_data_from_jointld(file_path)

# Print the extracted data
for entry in data:
    print(f"Index: {entry['index']}")
    print(f"Goal Positions: {entry['dxl_goal_position']}")
    print(f"Goal Velocities: {entry['dxl_goal_velocity']}")
    print('-' * 50)
