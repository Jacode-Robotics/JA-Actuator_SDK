% 打开文件
fileID = fopen('JointLD_25.data', 'r');
fileID_out = fopen('JointLD_25_modified.data', 'w'); % 创建一个新文件用于存储修改后的数据

% 初始化变量
index_values = [];
dxl_goal_positions = [];
dxl_goal_velocities = [];

% 逐行读取文件
tline = fgetl(fileID);
first_line = true;  % 标记是否为第一行
prev_position = [];  % 用于累加的初始位置值
time_interval = 0.008; % 8ms = 0.008s

while ischar(tline)
    % 匹配格式 |index; 后跟浮点数的字符串
    expression = '\|(\d+);([\d\.\-\+,]+)';
    match = regexp(tline, expression, 'tokens');
    
    if ~isempty(match)
        index = str2double(match{1}{1});
        data_values = str2num(match{1}{2}); %#ok<ST2NM> % 转换为数值数组
        
        % 确保数据长度为12
        if length(data_values) == 12
            % 提取原始位置和速度
            dxl_goal_position = data_values(1:6);
            dxl_goal_velocity = data_values(7:12);
            
            % 将速度绝对值小于1.0的值设为0
            dxl_goal_velocity(abs(dxl_goal_velocity) < 1.0) = 0;
            
            % 如果是第一行，保存初始位置
            if first_line
                prev_position = dxl_goal_position; % 保存第一行位置
                first_line = false;
            else
                % 通过速度累加位置，公式：position_new = position_old + velocity * time_interval
                dxl_goal_position = prev_position + dxl_goal_velocity * time_interval;
                prev_position = dxl_goal_position; % 更新为当前行的累积位置
            end
            
            % 存储修改后的结果
            index_values = [index_values; index]; %#ok<AGROW>
            dxl_goal_positions = [dxl_goal_positions; dxl_goal_position]; %#ok<AGROW>
            dxl_goal_velocities = [dxl_goal_velocities; dxl_goal_velocity]; %#ok<AGROW>
            
            % 将修改后的数据写回新文件
            fprintf(fileID_out, '|%d;%s\n', index, ...
                strjoin(arrayfun(@(x) num2str(x), [dxl_goal_position, dxl_goal_velocity], 'UniformOutput', false), ','));
        end
    end
    
    % 读取下一行
    tline = fgetl(fileID);
end

% 关闭文件
fclose(fileID);
fclose(fileID_out);

% 颜色定义
colors = lines(6);

% 绘制位置数据
figure;
hold on;
for i = 1:6
    plot(index_values, dxl_goal_positions(:, i), 'Color', colors(i,:), 'DisplayName', ['Position ' num2str(i)]);
end
xlabel('Index');
ylabel('Cumulative Goal Position (degree)');
title('Cumulative Goal Positions');
legend('show');
grid on;

% 绘制速度数据
figure;
hold on;
for i = 1:6
    plot(index_values, dxl_goal_velocities(:, i), 'Color', colors(i,:), 'DisplayName', ['Velocity ' num2str(i)]);
end
xlabel('Index');
ylabel('Goal Velocity (degree/s)');
title('Filtered Goal Velocities');
legend('show');
grid on;
