classdef UAVAgent< handle
    properties(Constant)
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
        current_target_id
        ukf_list            % {1×N} cell，每个敌人的 UKF
        estimated_states    % {1×N} cell，每个敌人的状态估计结果（6×1）
        combat_states       % {1×N} cell，每个敌人的 [bearing; angle_off; dist]
        evaluators              % {1×N} 每个敌人的 CombatEvaluator 实例
        situation_posterior     % {1×N} 每个敌人的后验概率（1×4）
        situation_utilities     % {1×N} 每个敌人的效用值（1×4）
    end

    methods
        % 构造函数
        function obj = UAVAgent(id,x,y,z,v,psi,gamma)
            obj.id = id;
            obj.state = [x; y; z; v; psi; gamma];
        end
        % 状态更新
        function obj = updata(obj, action_id)
            % 获取当前状态
            curr = obj.state(:, end);
            x = curr(1); y = curr(2); z = curr(3);
            v = curr(4); psi = curr(5); gamma = curr(6);
            dtt = obj.dt; g0 = obj.g;
            % 控制输入
            u = obj.maneuver_lib(action_id, :)';  % [nx; ny; gamma_c]
            % 状态更新
            x = x + v * cos(gamma) * cos(psi) * dtt;
            y = y + v * cos(gamma) * sin(psi) * dtt;
            z = z + v * sin(gamma) * dtt;
            v = v + g0 * u(1) * dtt;
            psi = psi + (g0 * u(2) / max(v, 1e-3)) * dtt;
            gamma = gamma + (g0 * tan(u(3)) / max(v, 1e-3)) * dtt;
            % 新状态列
            new_state = [x; y; z; v; wrapToPi(psi); gamma];
            % === 追加 ===
            obj.state(:, end+1) = new_state;
        end

        % 战斗态势
        function [bearing, angle_off, dist] = fight_state(obj, enemy)
            self_pos = obj.state(1:3);
            enemy_pos = enemy.state(1:3);
            rel_vec = enemy_pos(:) - self_pos(:);
            dist = norm(rel_vec);

            % 自己方向
            psi = obj.state(5); gamma = obj.state(6);
            self_vec = [cos(gamma)*cos(psi); cos(gamma)*sin(psi); sin(gamma)];

            % 敌人方向
            psi_e = enemy.state(5); gamma_e = enemy.state(6);
            enemy_vec = [cos(gamma_e)*cos(psi_e); cos(gamma_e)*sin(psi_e); sin(gamma_e)];

            % 保证都是列向量
            self_vec = self_vec(:);
            enemy_vec = enemy_vec(:);

            % dot 操作
            bearing = acos(dot(rel_vec, self_vec) / (dist + 1e-6));
            angle_off = acos(dot(-rel_vec, enemy_vec) / (dist + 1e-6));
        end

        % 初始化用于某个敌人的 UKF
        function obj = init_ukf(obj, enemy_state)
            % 参数说明：
            %   enemy_state - 6×1 敌方 UAV 的真实状态（用于初始化 UKF）

            % === 参数设置 ===
            state_dim = 6;
            obs_dim = 3;
            Q = 0.1 * eye(state_dim);   % 过程噪声协方差
            R = 10 * eye(obs_dim);      % 观测噪声协方差
            alpha = 1e-3; beta = 2; kappa = 1e-3;

            % === 创建 UKF 对象 ===
            ukf_obj = ukf(state_dim, obs_dim, Q, R, alpha, beta, kappa);

            % === 设置 UKF 初始值 ===
            ukf_obj.x = enemy_state;
            ukf_obj.P = eye(state_dim);

            % === 加入列表 ===
            new_index = length(obj.ukf_list) + 1;
            obj.ukf_list{new_index} = ukf_obj;
            obj.estimated_states{new_index} = enemy_state;
            obj.combat_states{new_index} = [0; 0; 0];  % 先占位
        end


        % 遍历每个敌人，观测 + UKF 更新 + 战斗态势评估
        function obj = estimate_enemies(obj)

            for i = 1:length(obj.enemys)
                enemy = obj.enemys{i};       % 当前敌机
                ukf_i = obj.ukf_list{i};     % 对该敌机的 UKF

                % === Step 1: 获取观测（真实敌人 -> 当前无人机的观测值）
                [bearing, angle_off, dist] = obj.fight_state(enemy);
                z = [bearing; angle_off; dist];

                % === Step 2: 定义观测函数 H(x)
                h = @(x) default_observation(obj.state(:,end),x);  % x = 6×1，估计状态

                % === Step 3: 状态预测（可选简化为恒等）
                f = @(x) default_transition(x, obj.dt);
                ukf_i = ukf_i.predict(f);

                % === Step 4: 状态更新
                ukf_i = ukf_i.update(z, h);
                x_est = ukf_i.x;

                % === Step 5: 保存结果
                obj.ukf_list{i} = ukf_i;
                obj.estimated_states{i} = x_est;
                obj.combat_states{i} = [bearing; angle_off; dist];
            end

            % 假设你为每个敌人都创建了一个 CombatEvaluator 实例
            % 比如 obj.evaluators{i} = CombatEvaluator();

            for i = 1:length(obj.enemys)
                % x_est = obj.estimated_states{i};           % 估计状态（6×1）

                bearing = obj.combat_states{i}(1);         % 观测 bearing
                angle_off = obj.combat_states{i}(2);       % 观测 angle_off
                dist = obj.combat_states{i}(3);            % 观测 dist

                % === Step 1: 更新后验概率 ===
                evaluator = obj.evaluators{i};             % CombatEvaluator 实例
                [posterior, evaluator] = evaluator.compute_posterior(bearing, angle_off, dist);
                obj.evaluators{i} = evaluator;             % 更新对象中的 evaluator

                % === Step 2: 计算各态势下的效用 ===
                utilities = zeros(1, 4);  % 每个态势下的效用（不考虑动作选择）
                for s = 1:4
                    utilities(s) = evaluator.utility_given_state_action(s, bearing, angle_off, dist);
                end

                % === 你可以将 posterior 和 utilities 存下来，用于决策或分析 ===
                obj.situation_posterior{i} = posterior;
                obj.situation_utilities{i} = utilities;

            end
        end
        % 选择目标
        function obj = select_target(obj)
            num_enemies = length(obj.enemys);
            utility_scores = zeros(1, num_enemies);

            % Step 1: 计算每个敌机的最大态势效用
            for i = 1:num_enemies
                utilities = obj.situation_utilities{i};  % 1x4
                posterior = obj.situation_posterior{i};  % 1x4
                % 期望效用
                utility_scores(i) = sum(posterior .* utilities);
            end

            % Step 2: 判断是否首次选择
            if isempty(obj.current_target_id)
                [~, obj.current_target_id] = max(utility_scores);
                return;
            end

            % Step 3: 判断当前目标是否失效或效用过低
            current_score = utility_scores(obj.current_target_id);
            [best_score, best_idx] = max(utility_scores);

            should_switch = false;

            % 条件1：当前目标异常（例如距离过远或未估计）
            combat_state = obj.combat_states{obj.current_target_id};
            if any(isnan(combat_state)) || combat_state(3) > 30000
                should_switch = true;
            end

            % 条件2：新目标效用高出 1.3 倍
            if best_score > 1.3 * current_score
                should_switch = true;
            end

            if should_switch
                obj.current_target_id = best_idx;
            end
        end
        % 选择动作
        function obj = select_and_update_action(obj)
            if isempty(obj.current_target_id)
                action_id = 7;  % 默认保持不动
                obj = obj.updata(action_id);
                return;
            end

            idx = obj.current_target_id;
            evaluator = obj.evaluators{idx};
            combat_state = obj.combat_states{idx};

            bearing = combat_state(1);
            angle_off = combat_state(2);
            dist = combat_state(3);

            [action_id, ~] = evaluator.select_best_action(bearing, angle_off, dist);

            % 执行动作
            obj = obj.updata(action_id);
        end
    end
end



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