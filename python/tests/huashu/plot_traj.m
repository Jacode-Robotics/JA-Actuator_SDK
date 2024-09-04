% 读取CSV文件
filename = 'J190_25.csv';
data = readtable(filename, 'TextType', 'string');

% 定义帧头
frame_head = '#FF FF FD 00 FE 25 00 8B 34 02 04 00';

% 遍历文件的第五列，从第三行开始
numRows = height(data);
startRow = 3; % 从第三行开始
columnIndex = 5; % 第五列

figure; % 创建图像窗口
hold on; % 允许在同一图上绘制多个曲线

for i = startRow:numRows
    % 从表格中读取每一行的命令帧字符串
    frame = data{i, columnIndex}; % 假设命令帧在第五列
    frame_data = strrep(frame, ' ', ''); % 去除空格，便于解析

    % 检查帧头是否匹配
    if ~startsWith(frame_data, strrep(frame_head, ' ', ''))
        fprintf('Row %d skipped: Frame head mismatch.\n', i);
        continue; % 如果帧头不匹配，跳过这一行
    end

    % 确保 frame_data 是字符串类型，如果不是，转换为字符串
    frame_data = char(frame_data);

    % 提取关节数据
    joint1_hex = frame_data(28:35); % 关节1的参考位置
    joint2_hex = frame_data(39:46); % 关节2的参考位置
    joint3_hex = frame_data(47:54); % 关节3的参考位置
    joint4_hex = frame_data(55:62); % 关节4的参考位置
    joint5_hex = frame_data(63:70); % 关节5的参考位置
    joint6_hex = frame_data(71:78); % 关节6的参考位置

    % 按两个字符为一组将字符串分割成字节
    bytes = reshape(joint1_hex, 2, []);
    
    % 颠倒字节顺序，以两个字符为一组
    joint1_hex = flipud(bytes);

    % 将提取的数据从十六进制转换为十进制（LSB格式）
    joint_positions(1) = hex2dec(joint1_hex); % 忽略前两位joint id
    joint_positions(2) = hex2dec(joint2_hex(3:end));
    joint_positions(3) = hex2dec(joint3_hex(3:end));
    joint_positions(4) = hex2dec(joint4_hex(3:end));
    joint_positions(5) = hex2dec(joint5_hex(3:end));
    joint_positions(6) = hex2dec(joint6_hex(3:end));

    % 对负数进行处理，若值大于2^31, 则为负数
    for j = 1:6
        if joint_positions(j) > 2^31 - 1
            joint_positions(j) = joint_positions(j) - 2^32;
        end
    end
    
    % 绘制关节位置图像
    joint_ids = 1:6; % 关节编号
    plot(joint_ids, joint_positions, '-o', 'LineWidth', 2, 'MarkerSize', 8);
end

% 图像标签和标题
xlabel('Joint ID');
ylabel('Reference Position (units)');
title('Joint Reference Positions from CSV (Starting from 3rd Row, 5th Column)');

% 添加图例
legend('show'); % 自动生成图例

hold off; % 结束图像绘制
