classdef CombatEvaluator < handle
    % CombatEvaluator: 战斗态势评估与动作选择器
    
    properties
        prior_state_prob  % 1x4 态势先验概率
        slope = 0.075     % 态势函数陡峭度
        dist_thresh = 20000;  % 距离阈值
        maneuver_lib      % 动作库（7×3）
        g = 9.8;           % 重力加速度
        dtt = 0.1;          % 时间步长（由外部设置）
    end

    methods
        function obj = CombatEvaluator()
            obj.prior_state_prob = ones(1, 4) / 4;
            obj.maneuver_lib = [
                3, 0, 0;
                -3, 0, 0;
                0, 5, 0;
                0, -5, 0;
                0, 0, 1.05;
                0, 0, -1.05;
                0, 0, 0
            ];
        end

        %% 观测似然值（对应论文表3）
        function p = likelihood_given_state(obj, bear, angle, dist, situation_id)
            ai = obj.slope;
            D = obj.dist_thresh;
            bearing_angle_13 = (ai*bear/pi + 1 - ai/2) / pi;
            bearing_angle_24 = (-ai*bear/pi + 1 + ai/2) / pi;
            angle_off_12 = (-ai*angle/pi + 1 + angle/2) / pi;
            angle_off_34 = (ai*angle/pi + 1 - angle/2) / pi;
            distance_1 = 1 / D;
            distance_234 = (-ai*dist/D + 1 + ai/2) / D;

            switch situation_id
                case 1
                    pb = bearing_angle_13;
                    pa = angle_off_12;
                    pd = distance_1;
                case 2
                    pb = bearing_angle_24;
                    pa = angle_off_12;
                    pd = distance_234;
                case 3
                    pb = bearing_angle_13;
                    pa = angle_off_34;
                    pd = distance_234;
                case 4
                    pb = bearing_angle_24;
                    pa = angle_off_34;
                    pd = distance_234;
            end
            p = pb * pa * pd;
        end

        %% 贝叶斯后验（式7）
        function [posterior, obj] = compute_posterior(obj, bearing, angle_off, dist)
            likelihoods = zeros(1, 4);
            for i = 1:4
                likelihoods(i) = obj.likelihood_given_state(bearing, angle_off, dist, i);
            end
            unnormalized = likelihoods .* obj.prior_state_prob;
            posterior = unnormalized / (sum(unnormalized) + 1e-6);
            obj.prior_state_prob = posterior;  % 更新
        end

        %% 单态势下效用（式11 + 表4）
        function u = utility_given_state_action(obj, s_id, bearing, angle, dist)
            D = obj.dist_thresh;
            switch s_id
                case 1
                    w = [0.2, 0.1, 0.7];
                    f = [(pi - bearing)/pi, (pi - angle)/pi, (D - dist)/D];
                case 2
                    w = [0.3, 0.0, 0.7];
                    f = [(pi - bearing)/pi, (pi - angle)/pi, (D - dist)/D];
                case 3
                    w = [0.0, 0.3, 0.7];
                    f = [bearing/pi, (pi - angle)/pi, dist/D];
                case 4
                    w = [0.1, 0.2, 0.7];
                    f = [(pi - bearing)/pi, (pi - angle)/pi, dist/D];
            end
            u = sum(w .* f);
        end

        %% 一阶期望效用（式27）
        function J = expected_utility(obj, bearing, angle, dist)
            [posterior, obj] = obj.compute_posterior(bearing, angle, dist);
            J = 0;
            for s = 1:4
                J = J + posterior(s) * obj.utility_given_state_action(s, bearing, angle, dist);
            end
        end

        %% 遍历所有动作，计算每个动作后的预测效用
        function [utilities] = evaluate_all_actions(obj, self_state, enemy_state)
            num_actions = size(obj.maneuver_lib, 1);
            utilities = zeros(1, num_actions);

            for a = 1:num_actions
                pred_state = obj.predict_state(self_state, a);
                [bearing, angle, dist] = obj.compute_combat_state(pred_state, enemy_state);
                utilities(a) = obj.expected_utility(bearing, angle, dist);
            end
        end

        %% 选择最优动作（返回动作编号和效用）
        function [best_action, utilities] = select_best_action(obj, self_state, enemy_state)
            utilities = obj.evaluate_all_actions(self_state, enemy_state);
            [~, best_action] = max(utilities);
        end

        %% 预测执行动作a后的状态
        function new_state = predict_state(obj, state, action_id)
            x = state(1); y = state(2); z = state(3);
            v = state(4); psi = state(5); gamma = state(6);
            u = obj.maneuver_lib(action_id, :)';
            g0 = obj.g; dt = obj.dtt;

            new_x = x + v * cos(gamma) * cos(psi) * dt;
            new_y = y + v * cos(gamma) * sin(psi) * dt;
            new_z = z + v * sin(gamma) * dt;
            new_v = v + g0 * u(1) * dt;
            new_psi = psi + (g0 * u(2) / max(new_v, 1e-3)) * dt;
            new_gamma = gamma + (g0 * tan(u(3)) / max(new_v, 1e-3)) * dt;

            new_state = [new_x; new_y; new_z; new_v; wrapToPi(new_psi); new_gamma];
        end

        %% 根据预测状态计算战斗态势
        function [bearing, angle, dist] = compute_combat_state(~, self_state, enemy_state)
            pos_self = self_state(1:3);
            pos_enemy = enemy_state(1:3);
            rel = pos_enemy - pos_self;
            dist = norm(rel);

            psi = self_state(5); gamma = self_state(6);
            vec_self = [cos(gamma)*cos(psi); cos(gamma)*sin(psi); sin(gamma)];

            psi_e = enemy_state(5); gamma_e = enemy_state(6);
            vec_enemy = [cos(gamma_e)*cos(psi_e); cos(gamma_e)*sin(psi_e); sin(gamma_e)];

            bearing = acos(dot(rel, vec_self) / (dist + 1e-6));
            angle = acos(dot(-rel, vec_enemy) / (dist + 1e-6));
        end
    end
end

