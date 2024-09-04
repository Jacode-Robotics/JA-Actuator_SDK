% 读取CSV文件并定义参数
filename = '20240904_195939_log.csv';
data = readtable(filename, 'TextType', 'string');

% 定义Reference, Feedback和Velocity Reference的帧头
frame_head_ref = '#FF FF FD 00 FE 25 00 8B 34 02 04 00';  % Reference帧头
frame_head_fdb = '#FF FF FD 00 FE 31 00 55';              % Feedback帧头
frame_head_vel_ref = '#FF FF FD 00 FE 25 00 83 28 02 04'; % Velocity Reference帧头

numRows = height(data);
startRow = 3;
columnIndex = 5;  % Frame data
timeColumnIndex = 2; % Assuming second column contains time
num_joints = 6;
joint_pos_ref = [];
joint_pos_fdb = [];  % Feedback数据
vel_ref = [];        % Velocity Reference数据
time_ref = []; % Time for the reference data

% 初始化上一个有效数据
prev_valid_ref = zeros(1, num_joints);
prev_valid_fdb = zeros(1, num_joints);
prev_valid_vel_ref = zeros(1, num_joints);  % Initialize for vel_ref

% 遍历数据，提取关节位置和时间
for i = startRow:numRows
    frame = data{i, columnIndex};
    frame_data = strrep(frame, ' ', ''); % 去除空格
    
    if startsWith(frame_data, strrep(frame_head_ref, ' ', ''))
        % 提取时间数据并转换为秒
        time_str = data{i, timeColumnIndex};  % Get the time string
        time_value = convert_time_to_seconds(time_str);  % Convert to seconds
        time_ref = [time_ref; time_value];  % Append to the time reference array
        
        % 处理Reference帧头
        current_ref = parse_frame_data(frame_data, num_joints, 28, 10);
        
        % No abnormality checks for current_ref
        joint_pos_ref = [joint_pos_ref; current_ref];
        
    elseif startsWith(frame_data, strrep(frame_head_fdb, ' ', ''))
        % 处理Feedback帧头
        current_fdb = parse_frame_data(frame_data, num_joints, 22, 16);
        
        % 检查是否异常，若变化大于1024则使用上一时刻数据
        for j = 1:num_joints
            if abs(current_fdb(j)) > 32768
                fprintf('Row %d, Joint %d: Feedback data anomaly detected, using previous valid data.\n', i, j);
                current_fdb(j) = prev_valid_fdb(j); % 使用上一时刻的有效数据
            else
                prev_valid_fdb(j) = current_fdb(j); % 更新有效数据
            end
        end
        joint_pos_fdb = [joint_pos_fdb; current_fdb];
        
    elseif startsWith(frame_data, strrep(frame_head_vel_ref, ' ', ''))
        % 处理Velocity Reference帧头
        current_vel_ref = parse_frame_data(frame_data, num_joints, 28, 10);
        
        % No abnormality checks for vel_ref data
        vel_ref = [vel_ref; current_vel_ref];
        
    else
        fprintf('Row %d skipped: Frame head mismatch.\n', i);
        continue;
    end
end

% 确保Reference和Feedback的行数相同
min_len = min([size(joint_pos_ref, 1), size(joint_pos_fdb, 1), size(vel_ref, 1)]);
joint_pos_ref = joint_pos_ref(1:min_len, :);
joint_pos_fdb = joint_pos_fdb(1:min_len, :);
vel_ref = vel_ref(1:min_len, :);
time_ref = time_ref(1:min_len);  % Match time with data length

% 将时间轴设置为以第一个数据为起始
time_ref = time_ref - time_ref(1);  % Subtract the first time value from all entries

% 计算误差
joint_error = joint_pos_ref - joint_pos_fdb;

% 绘制Reference和Feedback数据
figure;
for j = 1:num_joints
    subplot(3, 2, j); % 3行2列的子图
    hold on;
    
    % 主坐标轴绘制Reference和Feedback数据，使用典型SCI配色和线型
    yyaxis left;
    plot(time_ref, joint_pos_ref(:, j), '-', 'LineWidth', 2, 'Color', [0 0.4470 0.7410], 'DisplayName', 'Reference'); % 蓝色实线
    plot(time_ref, joint_pos_fdb(:, j), '--', 'LineWidth', 2, 'Color', [0 0.8500 0.0980], 'DisplayName', 'Feedback'); % 红色虚线
    ylabel(sprintf('Position (counts)'), 'FontSize', 12, 'Interpreter', 'latex');
    
    % 次坐标轴绘制误差
    yyaxis right;
    plot(time_ref, joint_error(:, j), '-.', 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'Error'); % 橙色点划线
    ylabel('Error (counts)', 'FontSize', 12, 'Interpreter', 'latex');
    
    % 设置图像样式
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1.5);
    xlabel('Time (seconds)', 'FontSize', 12, 'Interpreter', 'latex');  % Use time in seconds
    title(sprintf('Joint %d', j), 'FontSize', 14, 'Interpreter', 'latex');
    
    % 设置图例
    legend({'Reference', 'Feedback', 'Error'}, 'Location', 'northeast', 'FontSize', 10);
    
    % 优化坐标轴
    box on; % 保留边框
    grid on; % 打开网格
    axis tight; % 自动调整坐标轴
    
    % 增加纵坐标的留白，留出上下10%的空白区域
    ylim padded;
    
    % 启用次刻度
    set(gca, 'XMinorTick', 'on', 'YMinorTick', 'on'); 
end

% 新建窗口，绘制Velocity Reference数据
figure;
for j = 1:num_joints
    subplot(3, 2, j); % 3行2列的子图
    hold on;
    
    % 绘制Velocity Reference数据
    plot(time_ref, vel_ref(:, j), '-', 'LineWidth', 2, 'Color', [0 0.4470 0.7410], 'DisplayName', 'Velocity Reference'); % 蓝色实线
    ylabel(sprintf('Velocity (0.01 rev/min)'), 'FontSize', 12, 'Interpreter', 'latex');
    
    % 设置图像样式
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1.5);
    xlabel('Time (seconds)', 'FontSize', 12, 'Interpreter', 'latex');  % Use time in seconds
    title(sprintf('Joint %d', j), 'FontSize', 14, 'Interpreter', 'latex');
    
    % 设置图例
    legend({'Velocity Reference'}, 'Location', 'northeast', 'FontSize', 10);
    
    % 优化坐标轴
    box on; % 保留边框
    grid on; % 打开网格
    axis tight; % 自动调整坐标轴
    
    % 增加纵坐标的留白，留出上下10%的空白区域
    ylim padded;
    
    % 启用次刻度
    set(gca, 'XMinorTick', 'on', 'YMinorTick', 'on'); 
end
