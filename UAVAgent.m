classdef UAVAgent < handle
    properties (Constant)
        g = 9.8;
        maneuver_lib = [
            3, 0, 0;
            -3, 0, 0;
            0, 5, 0;
            0, -5, 0;
            0, 0, 1.05;
            0, 0, -1.05;
            0, 0, 0
        ];
    end

    properties
        state       % [x; y; z; v; psi; gamma]
        dt = 0.1;
        id
        enemys
        current_target_id = [];
        uold = [0, 0, 0];
        ukf_list            % {1×N} cell，每个敌人的 UKF
        estimated_states    % {1×N} cell，每个敌人的状态估计
        combat_states       % {1×N} cell，每个敌人的 [bearing; angle_off; dist]
        evaluators          % {1×N} 每个敌人的 CombatEvaluator 实例
        situation_posterior % {1×N} 态势后验概率（1×4）
        situation_utilities % {1×N} 各态势的效用（1×4）
    end

    methods
        function obj = UAVAgent(id,x,y,z,v,psi,gamma)
            obj.id = id;
            obj.state = [x; y; z; v; psi; gamma];
        end
%% 状态更新
        function obj = updata(obj, action_id)
            curr = obj.state(:, end);
            x = curr(1); y = curr(2); z = curr(3);
            v = curr(4); psi = curr(5); gamma = curr(6);
            dtt = obj.dt;

            % 控制输入处理
            u = 0.25 * obj.maneuver_lib(action_id, :) + obj.uold;
            u = max(min(u, [3, 5, 1.05]), [-3, -5, -1.05]);  % 限幅
            obj.uold = u;

            % 状态更新
            x = x + v * cos(gamma) * cos(psi) * dtt;
            y = y + v * cos(gamma) * sin(psi) * dtt;
            z = z + v * sin(gamma) * dtt;
            v = v + obj.g * u(1) * dtt;
            v = min(v,408);
            psi = psi + (obj.g * u(2) / max(v, 1e-3)) * dtt;
            gamma = gamma + (obj.g * tan(u(3)) / max(v, 1e-3)) * dtt;

            new_state = [x; y; z; v; wrapToPi(psi); gamma];
            obj.state(:, end+1) = new_state;
        end
%% 初始化UKF估计器
        function obj = init_ukf(obj, enemy_state)
            state_dim = 6; obs_dim = 3;
            Q = 0.1 * eye(state_dim);
            R = 10 * eye(obs_dim);
            ukf_obj = ukf(state_dim, obs_dim, Q, R, 1e-3, 2, 1e-3);
            ukf_obj.x = enemy_state;
            ukf_obj.P = eye(state_dim);

            new_index = length(obj.ukf_list) + 1;
            obj.ukf_list{new_index} = ukf_obj;
            obj.estimated_states{new_index} = enemy_state;
            obj.combat_states{new_index} = [0; 0; 0];
        end
%% 估计敌方位置、态势
        function obj = estimate_enemies(obj)
            for i = 1:length(obj.enemys)
                enemy = obj.enemys{i};
                ukf_i = obj.ukf_list{i};

                [bearing, angle_off, dist] = compute_predicted_combat_state(obj.state(:,end), enemy.state(:,end));
                z = [bearing; angle_off; dist];

                h = @(x) default_observation(obj.state(:,end), x);
                f = @(x) default_transition(x, obj.dt);

                ukf_i = ukf_i.predict(f);
                ukf_i = ukf_i.update(z, h);

                x_est = ukf_i.x;
                obj.ukf_list{i} = ukf_i;
                obj.estimated_states{i} = x_est;
                obj.combat_states{i} = [bearing; angle_off; dist];

                evaluator = obj.evaluators{i};
                evaluator.dtt = obj.dt;  % 确保时步一致
                [posterior, evaluator] = evaluator.compute_posterior(bearing, angle_off, dist);
                obj.evaluators{i} = evaluator;

                utilities = zeros(1, 4);
                for s = 1:4
                    utilities(s) = evaluator.utility_given_state_action(s, bearing, angle_off, dist);
                end
                obj.situation_posterior{i} = posterior;
                obj.situation_utilities{i} = utilities;
            end
        end
%% 选择敌机
        function obj = select_target(obj)
            num_enemies = length(obj.enemys);
            utility_scores = zeros(1, num_enemies);

            for i = 1:num_enemies
                utilities = obj.situation_utilities{i};
                posterior = obj.situation_posterior{i};
                utility_scores(i) = sum(posterior .* utilities);
            end

            if isempty(obj.current_target_id)
                [~, obj.current_target_id] = max(utility_scores);
                return;
            end

            current_score = utility_scores(obj.current_target_id);
            [best_score, best_idx] = max(utility_scores);

            combat_state = obj.combat_states{obj.current_target_id};
            if any(isnan(combat_state)) || combat_state(3) > 30000
                obj.current_target_id = best_idx;
                return;
            end

            if best_score > 1.3 * current_score
                obj.current_target_id = best_idx;
            end
        end
        %% 选择最佳动作
        function obj = select_and_update_action(obj)
            if isempty(obj.current_target_id)
                obj = obj.updata(7);  % 默认动作：保持不动
                return;
            end

            idx = obj.current_target_id;
            enemy = obj.enemys{idx};
            evaluator = obj.evaluators{idx};
            evaluator.dtt = obj.dt;

            self_state = obj.state(:, end);
            enemy_state = enemy.state(:, end);

            [best_action, ~] = evaluator.select_best_action(self_state, enemy_state);
            obj = obj.updata(best_action);
        end
    end
end


%% ukf状态更新
function x_next = default_transition(x, dt)
% x: 当前状态 [x; y; z; v; psi; gamma]
% dt: 时间步长
% g: 重力加速度（用于一致性
v = x(4);
psi = x(5);
gamma = x(6);

% 速度方向向量
dx = v * cos(gamma) * cos(psi);
dy = v * cos(gamma) * sin(psi);
dz = v * sin(gamma);

% 位置更新
x_next = x;
x_next(1) = x(1) + dx * dt;
x_next(2) = x(2) + dy * dt;
x_next(3) = x(3) + dz * dt;

% 其他状态不变（可拓展：加入模型噪声或控制项）
end
%% ukf观测函数
function z = default_observation(x_self, x_enemy)
% x_self: 己方状态 [x; y; z; v; psi; gamma]
% x_enemy: 敌方状态 [x; y; z; v; psi; gamma]

% 位置向量
p_self = x_self(1:3);
p_enemy = x_enemy(1:3);
rel = p_enemy - p_self;
dist = norm(rel);

% 自己方向向量
psi = x_self(5); gamma = x_self(6);
self_vec = [cos(gamma)*cos(psi);
    cos(gamma)*sin(psi);
    sin(gamma)];

% 敌人方向向量
psi_e = x_enemy(5); gamma_e = x_enemy(6);
enemy_vec = [cos(gamma_e)*cos(psi_e);
    cos(gamma_e)*sin(psi_e);
    sin(gamma_e)];

% bearing：敌人相对于己方朝向的夹角
bearing = acos(dot(rel, self_vec) / (dist + 1e-6));

% angle-off：敌人朝向与相对向量的夹角
angle_off = acos(dot(-rel, enemy_vec) / (dist + 1e-6));

z = [bearing; angle_off; dist];
end


 %% 辅助函数：根据预测状态计算战斗参数
function [bearing, angle_off, dist] = compute_predicted_combat_state(self_state, enemy_state)
    self_pos = self_state(1:3);
    enemy_pos = enemy_state(1:3);
    rel_vec = enemy_pos - self_pos;
    dist = norm(rel_vec);

    psi = self_state(5); gamma = self_state(6);
    self_vec = [cos(gamma)*cos(psi); cos(gamma)*sin(psi); sin(gamma)];

    psi_e = enemy_state(5); gamma_e = enemy_state(6);
    enemy_vec = [cos(gamma_e)*cos(psi_e); cos(gamma_e)*sin(psi_e); sin(gamma_e)];

    bearing = acos(dot(rel_vec, self_vec) / (dist + 1e-6));
    angle_off = acos(dot(-rel_vec, enemy_vec) / (dist + 1e-6));
end