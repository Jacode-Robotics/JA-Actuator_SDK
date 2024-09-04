% 函数定义必须放在脚本的最后
function joint_positions = parse_frame_data(frame_data, num_joints, start_offset, step_size)
    joint_positions = zeros(1, num_joints);
    frame_data = char(frame_data); % Convert string to character array
    for j = 1:num_joints
        start_idx = start_offset + (j - 1) * step_size; 
        end_idx = start_idx + 7; % Assume each joint position is represented by 8 characters (4 bytes)
        joint_hex = frame_data(start_idx:end_idx); % Extract the hexadecimal string
        split_groups = cellstr(reshape(joint_hex, 2, []).'); % Split the string into groups of 2 characters (1 byte)
        reversed_groups = flip(split_groups); % Reverse the byte order (for little-endian conversion)
        new_string = strcat(reversed_groups{:}); % Concatenate the reversed groups back into a single string
        joint_positions(j) = typecast(uint32(hex2dec(new_string)), 'int32'); % Convert hex to int32
    end
end
