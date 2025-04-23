clc;
clear;
% 初始化
RedUAV = cell(1,2);
BlueUAV = cell(1,2);

RedUAV{1} = UAVAgent(1,4250,8000,10000,300,0,90);
RedUAV{2} = UAVAgent(1,5250,9000,10000,300,0,90);

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
colors_red = lines(length(RedUAV));
colors_blue = lines(length(BlueUAV));

% 绘制红方 UAV
for i = 1:length(RedUAV)
    traj = RedUAV{i}.state(1:3, :);   % 取出位置轨迹
    x = traj(1, :); y = traj(2, :); z = traj(3, :)/100;
    plot3(x, y, z, '-', 'Color', colors_red(i,:), 'LineWidth', 2);
    scatter3(x(1), y(1), z(1), 60, 'go', 'filled');      % 起点
    scatter3(x(end), y(end), z(end), 60, 'ro', 'filled'); % 终点
end

% 绘制蓝方 UAV
for i = 1:length(BlueUAV)
    traj = BlueUAV{i}.state(1:3, :);
    x = traj(1, :); y = traj(2, :); z = traj(3, :);
    plot3(x, y, z, '--', 'Color', colors_blue(i,:), 'LineWidth', 2);
    scatter3(x(1), y(1), z(1), 60, 'go', 'filled');
    scatter3(x(end), y(end), z(end), 60, 'ro', 'filled');
end

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('UAV Flight Trajectories');
grid on;
axis equal;
view(3);
legend({'RedUAV 1', 'RedUAV 2', 'BlueUAV 1', 'BlueUAV 2'}, 'Location', 'best');

