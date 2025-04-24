figure;
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('UAV Flight Trajectories (Step-by-step)');
view(2)

% 获取轨迹长度
T = size(RedUAV{1}.state, 2);

% 初始化 plot 句柄
red_lines = gobjects(length(RedUAV), 1);
blue_lines = gobjects(length(BlueUAV), 1);

for i = 1:length(RedUAV)
    red_lines(i) = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 2);
end
for i = 1:length(BlueUAV)
    blue_lines(i) = plot3(NaN, NaN, NaN, 'b--', 'LineWidth', 2);
end

legend({'Red UAVs', 'Blue UAVs'}, 'Location', 'best');

% 动画循环
for t = 1:T
    for i = 1:length(RedUAV)
        traj = RedUAV{i}.state(1:3, 1:t);
        set(red_lines(i), 'XData', traj(1,:), 'YData', traj(2,:), 'ZData', traj(3,:));
    end
    for i = 1:length(BlueUAV)
        traj = BlueUAV{i}.state(1:3, 1:t);
        set(blue_lines(i), 'XData', traj(1,:), 'YData', traj(2,:), 'ZData', traj(3,:));
    end
    
    pause(0.05);  % 控制播放速度，越大越慢
end

% 绘制最终起止点
for i = 1:length(RedUAV)
    traj = RedUAV{i}.state(1:3, :);
    scatter3(traj(1,1), traj(2,1), traj(3,1), 60, 'g', 'filled');
    scatter3(traj(1,end), traj(2,end), traj(3,end), 60, 'r', 'filled');
end
for i = 1:length(BlueUAV)
    traj = BlueUAV{i}.state(1:3, :);
    scatter3(traj(1,1), traj(2,1), traj(3,1), 60, 'g', 'filled');
    scatter3(traj(1,end), traj(2,end), traj(3,end), 60, 'r', 'filled');
end