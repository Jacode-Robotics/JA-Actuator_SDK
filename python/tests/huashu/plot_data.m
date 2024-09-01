% 打开文件
fileID = fopen('JointLD_25.data', 'r');

% 初始化变量
index_values = [];
dxl_goal_positions = [];
dxl_goal_velocities = [];

% 逐行读取文件
tline = fgetl(fileID);
while ischar(tline)
    % 匹配格式 |index; 后跟浮点数的字符串
    expression = '\|(\d+);([\d\.\-\+,]+)';
    match = regexp(tline, expression, 'tokens');
    
    if ~isempty(match)
        index = str2double(match{1}{1});
        data_values = str2num(match{1}{2}); %#ok<ST2NM> % 转换为数值数组
        
        % 确保数据长度为12
        if length(data_values) == 12
            % 缩放位置和速度，并转换为整数
            dxl_goal_position = round((data_values(1:6) / 360.0) * 32768);
            dxl_goal_velocity = round((data_values(7:12) / 360.0) * 32768);
            
            % 存储结果
            index_values = [index_values; index]; %#ok<AGROW>
            dxl_goal_positions = [dxl_goal_positions; dxl_goal_position]; %#ok<AGROW>
            dxl_goal_velocities = [dxl_goal_velocities; dxl_goal_velocity]; %#ok<AGROW>
        end
    end
    
    % 读取下一行
    tline = fgetl(fileID);
end

% 关闭文件
fclose(fileID);

% 颜色定义
colors = lines(6);

% 绘制位置数据
figure;
hold on;
for i = 1:6
    plot(index_values, dxl_goal_positions(:, i), 'Color', colors(i,:), 'DisplayName', ['Position ' num2str(i)]);
end
xlabel('Index');
ylabel('Scaled Goal Position');
title('Goal Positions');
legend('show');
grid on;

% 绘制速度数据
figure;
hold on;
for i = 1:6
    plot(index_values, dxl_goal_velocities(:, i), 'Color', colors(i,:), 'DisplayName', ['Velocity ' num2str(i)]);
end
xlabel('Index');
ylabel('Scaled Goal Velocity');
title('Goal Velocities');
legend('show');
grid on;
