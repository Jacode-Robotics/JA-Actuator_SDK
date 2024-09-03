% 读取CSV文件的内容
filename = '20240817_014104_log.csv';
% filename = '20240817_010948_log.csv';
data = readtable(filename, 'Delimiter', ',', 'ReadVariableNames', false);

% 初始化变量
referencePosition = [];
positionFeedback = [];
timeAxis = [];
lastReferencePosition = 0; % 记录上次提取的 reference position
lastPositionFeedback = 0;  % 记录上次提取的 position feedback

% 获取第二行的起始时间，并去掉'#'符号
startTimeStr = strrep(data{2, 2}{1}, '#', ''); % 去除'#'符号
startTime = datetime(startTimeStr, 'InputFormat', 'HH:mm:ss.SSS');

% 从第二行开始遍历每一行的数据
for i = 2:height(data)
    % 提取当前行的第五列的字符串
    targetString = data{i, 5}{1};
    


    % 查找目标字符串 "8B 34 02 04 00 01"
    pattern1 = '8B 34 02 04 00 01';
    % pattern1 = '8B 28 02 04 00 01';
    pattern2 = '55 00 01';
    startIndex1 = strfind(targetString, pattern1);
    startIndex2 = strfind(targetString, pattern2);
    
    if ~isempty(startIndex1)
    flag = true;

    % 提取当前行的第二列时间字符串并去掉'#'符号
    currentTimeStr = strrep(data{i, 2}{1}, '#', ''); % 去除'#'符号
    currentTime = datetime(currentTimeStr, 'InputFormat', 'HH:mm:ss.SSS');
    
    % 计算相对起始时间的秒数
    elapsedTime = seconds(currentTime - startTime);
    timeAxis = [timeAxis; elapsedTime];

        % 提取 "8B 34 02 04 00 01" 之后的13个字符
        hexString = targetString(startIndex1 + length(pattern1):startIndex1 + length(pattern1) + 12);
        
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
        
        % 更新 reference position
        lastReferencePosition = signedValue;
        
    % 确保 reference position 和 position feedback 的数量一致
    referencePosition = [referencePosition; lastReferencePosition];
    positionFeedback = [positionFeedback; lastPositionFeedback];

    elseif ~isempty(startIndex2) & flag
        flag = false;
    % 提取当前行的第二列时间字符串并去掉'#'符号
    currentTimeStr = strrep(data{i, 2}{1}, '#', ''); % 去除'#'符号
    currentTime = datetime(currentTimeStr, 'InputFormat', 'HH:mm:ss.SSS');
    
    % 计算相对起始时间的秒数
    elapsedTime = seconds(currentTime - startTime);
    timeAxis = [timeAxis; elapsedTime];

        % 提取 "55 00 01" 之后的13个字符
        hexString = targetString(startIndex2 + length(pattern2):startIndex2 + length(pattern2) + 12);
        
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
        
        % 更新 position feedback
        lastPositionFeedback = signedValue;

    % 确保 reference position 和 position feedback 的数量一致
    referencePosition = [referencePosition; lastReferencePosition];
    positionFeedback = [positionFeedback; lastPositionFeedback];
    else
        flag = false;
    end
    

end

% 计算 position error
positionError = referencePosition - positionFeedback;

% 绘制在一个图中
figure;
yyaxis left
plot(timeAxis, referencePosition, '-b', 'DisplayName', 'Reference');
hold on;
plot(timeAxis, positionFeedback, '-r', 'DisplayName', 'Feedback');
xlabel('Time (seconds)');
ylabel('Trajectory');
title('Position Trajectory Tracking Performance');

yyaxis right
plot(timeAxis, positionError, '-g', 'DisplayName', 'Error');
ylabel('Error');

legend('show');
grid on;
hold off;
