function x_next = f_transition_acceleration(x, u, dt)
    % 状态变量 x: [x; y; z; vx; vy; vz]
    % 控制变量 u: [ax; ay; az]
    % dt: 时间步长
    A = eye(6);
    A(1,4) = dt; A(2,5) = dt; A(3,6) = dt;

    B = zeros(6,3);
    B(4,1) = dt; B(5,2) = dt; B(6,3) = dt;

    x_next = A * x + B * u;
end

function x_next = f_transition_aircraft(x, u, dt)
    % x = [x; y; z; v; psi; gamma]
    % u = [a_t; a_z; omega_psi]
    a_t = u(1); a_z = u(2); omega = u(3);

    v = x(4) + a_t * dt;
    v = max(v, 1);  % 避免为负
    psi = x(5) + omega * dt;
    gamma = x(6) + a_z * dt;

    dx = v * cos(gamma) * cos(psi) * dt;
    dy = v * cos(gamma) * sin(psi) * dt;
    dz = v * sin(gamma) * dt;

    x_next = [
        x(1) + dx;
        x(2) + dy;
        x(3) + dz;
        v;
        wrapToPi(psi);
        gamma
    ];
end

function z = h_observation_bearing_angle_dist(x, own_pos)
    % x: 敌机状态 [x; y; z; vx; vy; vz]
    % own_pos: 我方位置 [x; y; z]
    dx = x(1:3) - own_pos(:);
    dist = norm(dx);
    bearing = atan2(dx(2), dx(1));
    elevation = atan2(dx(3), norm(dx(1:2)));
    z = [bearing; elevation; dist];
end

% 初始化UKF
state_dim = 6; obs_dim = 3; dt = 1;
Q = 0.01 * eye(state_dim);
R = diag([deg2rad(2), deg2rad(2), 0.1]);  % 方位角、俯仰角、距离

ukf_filter = ukf(state_dim, obs_dim, Q, R, 1e-3, 2, 0);

% 初始估计
ukf_filter.x = [10; 10; 10; 0; 0; 0];
ukf_filter.P = 0.1 * eye(state_dim);

% 仿真敌人真实轨迹
true_state = [10; 10; 10; 5; 0; 0];  % 初始位置+速度
N = 50;  % 轨迹长度

trajectory_true = zeros(state_dim, N);
trajectory_est = zeros(state_dim, N);

for k = 1:N
    % 控制输入（可自定义）：
    if k < N
        u = [0.2; 0.1; 0.05];  % 加速
    else
        u = [0; 0; 0];
    end

    % === 选择模型 ===
    % 第一版：
    % true_state = f_transition_acceleration(true_state, u, dt);

    % 第二版（若使用）：
    true_state = f_transition_aircraft(true_state, u, dt);

    % 生成观测（带观测噪声）
    z_true = h_observation_bearing_angle_dist(true_state, [0; 0; 0]);
    z = z_true + mvnrnd(zeros(obs_dim,1), R)';

    % === UKF步骤 ===
    % ukf_filter = ukf_filter.predict(@(x) f_transition_acceleration(x, u, dt));
    ukf_filter = ukf_filter.predict(@(x) f_transition_aircraft(x, u, dt));  % 第二版

    ukf_filter = ukf_filter.update(z, @(x) h_observation_bearing_angle_dist(x, [0; 0; 0]));

    % 保存轨迹
    trajectory_true(:, k) = true_state;
    trajectory_est(:, k) = ukf_filter.x;
end

% 绘图
figure;
plot3(trajectory_true(1,:), trajectory_true(2,:), trajectory_true(3,:), 'b-', 'LineWidth', 2); hold on;
plot3(trajectory_est(1,:), trajectory_est(2,:), trajectory_est(3,:), 'r--', 'LineWidth', 2);
legend('真实轨迹', '估计轨迹');
xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
title('敌人轨迹估计（UKF）');