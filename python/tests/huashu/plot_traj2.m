frame_data = "#FFFFFFD00F25008834020400113000002860000003F6FFFFFF04E5FFFFFF05B0FFFFFF06C12F0000A916";

% 确保 frame_data 是字符串类型，如果不是，转换为字符串
frame_data = char(frame_data);

% 检查字符串长度
if length(frame_data) >= 44
    hex_string = frame_data(27:34); % 假设 13000000 从这个位置开始
    % 将十六进制字符串转换为十进制
    decimal_value = hex2dec(hex_string);
    % 显示结果
    disp(decimal_value);
else
    disp('frame_data 字符串长度不足，无法提取到13000000');
end
