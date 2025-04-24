clc;
clear;

% 创建 UAVAgent 对象
uav = UAVAgent(1,4250,8000,10000,300,0,90);

% 创建图形窗口
figure;
hold on; % 保持图形，以便在循环中绘制多条轨迹
view(3); % 设置为三维视图
% 初始化存储轨迹的变量
x = []; % 用于存储 x 坐标
y = []; % 用于存储 y 坐标
z = []; % 用于存储 z 坐标

% 循环更新 UAVAgent 的状态并绘制轨迹
for step = 1:14
    t = mod(step,7) + 1;
    uav.updata(t); % 更新 UAVAgent 的状态

    % 获取当前状态
    x = [x, uav.state(1,end)]; % 将新的 x 坐标追加到 x 中
    y = [y, uav.state(2,end)]; % 将新的 y 坐标追加到 y 中
    z = [z, uav.state(3,end)]; % 将新的 z 坐标追加到 z 中

    % 绘制三维轨迹
    plot3(x, y, z, 'b.-'); % 使用蓝色点线绘制轨迹

    % 设置图形属性
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('UAV Trajectory');
    grid on; % 添加网格
    drawnow; % 实时更新图形
end

% 保持图形
hold off;