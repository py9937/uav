clc;
clear;
% 初始化
RedUAV = cell(1,2);
BlueUAV = cell(1,2);

RedUAV{1} = UAVAgent(1,4250,8000,10000,300,-90,0);
RedUAV{2} = UAVAgent(1,5250,9000,10000,300,90,0);

BlueUAV{1} = UAVAgent(1,1250,1000,10000,300,0,0);
BlueUAV{2} = UAVAgent(1,2250,2500,10000,300,0,0);

for i = 1:length(RedUAV)
    RedUAV{i}.enemys = BlueUAV;
    for j = 1:length(BlueUAV)
        RedUAV{i}.init_ukf(BlueUAV{j}.state);
        RedUAV{i}.evaluators{j} = CombatEvaluator();
    end
end
for i = 1:length(BlueUAV)
    BlueUAV{i}.enemys = RedUAV;
    for j = 1:length(RedUAV)
        BlueUAV{i}.init_ukf(RedUAV{j}.state);
        BlueUAV{i}.evaluators{j} = CombatEvaluator();
    end
end

for step = 0.1 : 0.1 : 100
    % ukf预测敌机位置并根据敌机位置计算战斗状态
    for i = 1:length(BlueUAV)
        BlueUAV{i} = BlueUAV{i}.estimate_enemies();
        BlueUAV{i}.select_target();
        BlueUAV{i}.select_and_update_action();
    end
    for i = 1:length(RedUAV)
        RedUAV{i} = RedUAV{i}.estimate_enemies();
        RedUAV{i}.select_target();
        RedUAV{i}.select_and_update_action();
    end
    % 

end

figure;
hold on;

% 红蓝颜色定义
color_red = [1, 0, 0];      % 红色
color_blue = [0, 0.447, 0.741];  % MATLAB 默认蓝

% 红方 UAV
for i = 1:length(RedUAV)
    traj = RedUAV{i}.state(1:3, :);  % 轨迹是 3×T
    x = traj(1, :); y = traj(2, :); z = traj(3, :);

    if i == 1  % 只为第一个添加图例
        h_red = plot3(x, y, z, '-', 'Color', color_red, 'LineWidth', 2);
    else
        plot3(x, y, z, '-', 'Color', color_red, 'LineWidth', 2);
    end
    scatter3(x(1), y(1), z(1), 50, 'g', 'filled');  % 起点
    scatter3(x(end), y(end), z(end), 50, 'r', 'filled');  % 终点
end

% 蓝方 UAV
for i = 1:length(BlueUAV)
    traj = BlueUAV{i}.state(1:3, :);
    x = traj(1, :); y = traj(2, :); z = traj(3, :);

    if i == 1
        h_blue = plot3(x, y, z, '--', 'Color', color_blue, 'LineWidth', 2);
    else
        plot3(x, y, z, '--', 'Color', color_blue, 'LineWidth', 2);
    end
    scatter3(x(1), y(1), z(1), 50, 'g', 'filled');
    scatter3(x(end), y(end), z(end), 50, 'r', 'filled');
end

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('UAV Flight Trajectories');

% 图例只显示红蓝
legend([h_red, h_blue], {'Red UAVs', 'Blue UAVs'}, 'Location', 'best');

grid on;
axis equal;
view([45 30]);

% 自动动态设置坐标范围（可选）
all_x = []; all_y = []; all_z = [];
for i = 1:length(RedUAV)
    all_x = [all_x, RedUAV{i}.state(1, :)];
    all_y = [all_y, RedUAV{i}.state(2, :)];
    all_z = [all_z, RedUAV{i}.state(3, :)];
end
for i = 1:length(BlueUAV)
    all_x = [all_x, BlueUAV{i}.state(1, :)];
    all_y = [all_y, BlueUAV{i}.state(2, :)];
    all_z = [all_z, BlueUAV{i}.state(3, :)];
end

xlim([min(all_x)-500, max(all_x)+500]);
ylim([min(all_y)-500, max(all_y)+500]);
zlim([min(all_z)-500, max(all_z)+500]);

