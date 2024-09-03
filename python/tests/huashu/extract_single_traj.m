% 读取CSV文件的内容
filename = '位置轨迹.csv';
data = readtable(filename, 'Delimiter', ',', 'ReadVariableNames', false);

% 初始化变量
extractedData = [];
timeAxis = [];

% 获取第二行的起始时间，并去掉'#'符号
startTimeStr = strrep(data{2, 2}{1}, '#', ''); % 去除'#'符号
startTime = datetime(startTimeStr, 'InputFormat', 'HH:mm:ss.SSS');

% 从第二行开始遍历每一行的数据
for i = 2:height(data)
    % 提取当前行的第五列的字符串
    targetString = data{i, 5}{1};
    
    % 提取当前行的第二列时间字符串并去掉'#'符号
    currentTimeStr = strrep(data{i, 2}{1}, '#', ''); % 去除'#'符号
    currentTime = datetime(currentTimeStr, 'InputFormat', 'HH:mm:ss.SSS');
    
    % 计算相对起始时间的秒数
    elapsedTime = seconds(currentTime - startTime);
    timeAxis = [timeAxis; elapsedTime];

    % 查找目标字符串 "83 34 02 04 00 01"
    pattern = '83 34 02 04 00 01';
    startIndex = strfind(targetString, pattern);
    
    if ~isempty(startIndex)
        % 提取 "83 34 02 04 00 01" 之后的13个字符
        hexString = targetString(startIndex + length(pattern):startIndex + length(pattern) + 12);
        
        % 去掉空格
        hexString = strrep(hexString, ' ', '');
        
        % 按两个字符为一组将字符串分割成字节
        bytes = reshape(hexString, 2, []).';
        
        % 颠倒字节顺序，以两个字符为一组
        reversedBytes = flipud(bytes);
        
        % 将颠倒顺序后的字节重新组合成字符串
        reversedHexString = reshape(reversedBytes.', 1, []);  % 将转置后的字符矩阵展平为行向量
        
        % 将十六进制字符串转换为有符号整数
        hexValue = sscanf(reversedHexString, '%x');
        signedValue = typecast(uint32(hexValue), 'int32'); % 转换为有符号整数
        
        % 将结果添加到数组中
        extractedData = [extractedData; signedValue];
    end
end

% 绘制数据图
figure;
plot(timeAxis, extractedData);
xlabel('Time (seconds)');
ylabel('Reference Position (count)');
title('Trajectory tracking performance');
grid on;
