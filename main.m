clc;
clear;

% === 初始化参数 ===
RedUAV = cell(1, 2);
BlueUAV = cell(1, 2);

RedUAV{1} = UAVAgent(1, 4250, 8000, 10000, 300, pi/2, 0);
RedUAV{2} = UAVAgent(2, 5250, 9000, 10000, 300, pi/2, 0);

BlueUAV{1} = UAVAgent(3, 1250, 1000, 10000, 300, 0, 0);
BlueUAV{2} = UAVAgent(4, 2250, 2500, 10000, 300, 0, 0);

% === 设置敌人、初始化 UKF 和 CombatEvaluator ===
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

% === 终止条件设置 ===
bearing_thresh = pi / 6;
angle_thresh = pi;
dist_thresh = 1000;

% === 主仿真循环 ===
max_time = 60;
dt = 0.1;
terminated = false;

for t = 0:dt:max_time
    % 蓝方更新
    for i = 1:length(BlueUAV)
        BlueUAV{i} = BlueUAV{i}.estimate_enemies();
        BlueUAV{i}.select_target();
        BlueUAV{i}.select_and_update_action();
    end

    % 红方更新
    for i = 1:length(RedUAV)
        RedUAV{i} = RedUAV{i}.estimate_enemies();
        RedUAV{i}.select_target();
        RedUAV{i}.select_and_update_action();
    end

    % === 终止条件判定 ===
    for i = 1:length(RedUAV)
        tgt_id = RedUAV{i}.current_target_id;
        if isempty(tgt_id), continue; end
        combat_state = RedUAV{i}.combat_states{tgt_id};
        if all(combat_state < [bearing_thresh; angle_thresh; dist_thresh])
            fprintf('✅ Terminated at t = %.1f s: RedUAV %d reached target %d\n', ...
                t, i, tgt_id);
            terminated = true;
            break;
        end
    end

    if terminated
        break;
    end
end

% === 绘图 ===
figure;
hold on;
color_red = [1, 0, 0];
color_blue = [0, 0.447, 0.741];

for i = 1:length(RedUAV)
    traj = RedUAV{i}.state(1:3, :);
    plot3(traj(1, :), traj(2, :), traj(3, :), '-', 'Color', color_red, 'LineWidth', 2);
    scatter3(traj(1, 1), traj(2, 1), traj(3, 1), 50, 'g', 'filled');
    scatter3(traj(1, end), traj(2, end), traj(3, end), 50, 'r', 'filled');
end

for i = 1:length(BlueUAV)
    traj = BlueUAV{i}.state(1:3, :);
    plot3(traj(1, :), traj(2, :), traj(3, :), '--', 'Color', color_blue, 'LineWidth', 2);
    scatter3(traj(1, 1), traj(2, 1), traj(3, 1), 50, 'g', 'filled');
    scatter3(traj(1, end), traj(2, end), traj(3, end), 50, 'r', 'filled');
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('UAV Flight Trajectories');
legend({'Red Traj', 'Blue Traj'}, 'Location', 'best');
grid on; axis equal; view([45 30]);

% 坐标自适应
all_x = []; all_y = []; all_z = [];
for i = [RedUAV, BlueUAV]
    all_x = [all_x, i{1}.state(1, :)];
    all_y = [all_y, i{1}.state(2, :)];
    all_z = [all_z, i{1}.state(3, :)];
end
xlim([min(all_x)-500, max(all_x)+500]);
ylim([min(all_y)-500, max(all_y)+500]);
zlim([min(all_z)-500, max(all_z)+500]);