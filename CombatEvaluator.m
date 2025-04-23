classdef CombatEvaluator  < handle
    % CombatEvaluator: 战斗态势评估与动作选择器

    properties
        prior_state_prob  % 1x4 态势先验概率
        slope = 0.075     % 态势函数陡峭度
        dist_thresh = 20000;       % 距离阈值
    end

    methods
        % 初始化先验概率，均匀分布
        function obj = CombatEvaluator()
            obj.prior_state_prob = ones(1, 4) / 4;
        end
        % 态势s下的观测似然 9  with table 2
        function p = likelihood_given_state(obj, bear, angle, dist, situation_id)
            ai = obj.slope;
            D = obj.dist_thresh;
            bearing_angle_13 = (ai*bear/pi+1-ai/2)/pi;
            bearing_angle_24 = (-ai*bear/pi+1+ai/2)/pi;
            angle_off_12 = (-ai*angle/pi+1+angle/2)/pi;
            angle_off_34 = (ai*angle/pi+1-angle/2)/pi;
            distance_1 = 1/D;
            distance_234 = (-ai*dist/D+1+ai/2)/D;
            switch situation_id
                case 1  % Neutral
                    pb = bearing_angle_13;
                    pa = angle_off_12;
                    pd = distance_1;
                case 2  % Superiority
                    pb = bearing_angle_24;
                    pa = angle_off_12;
                    pd = distance_234;
                case 3  % Inferiority
                    pb = bearing_angle_13;
                    pa = angle_off_34;
                    pd = distance_234;
                case 4  % Mutual Threat
                    pb = bearing_angle_24;
                    pa = angle_off_34;
                    pd = distance_234;
            end
            p = pb * pa * pd;
        end
        % 计算后验态势概率 7
        function [posterior,obj] = compute_posterior(obj, bearing, angle_off, dist)
            likelihoods = zeros(1, 4);
            for i = 1:4
                likelihoods(i) = obj.likelihood_given_state(bearing, angle_off, dist, i);
            end
            unnormalized = likelihoods .* obj.prior_state_prob;
            posterior = unnormalized / (sum(unnormalized) + 1e-6);
            obj.prior_state_prob = posterior;  % 更新先验为后验
        end
        % 单态势下动作效用函数 11 with table3
        function u = utility_given_state_action(obj,situation_id,bearing, angle_off, dist )
            D = obj.dist_thresh;
            switch situation_id
                case 1
                    w = [0.2,0.1,0.7];
                    f = [(pi-bearing)/pi, (pi-angle_off)/pi, (D-dist)/D];
                case 2
                    w = [0.3, 0.0, 0.7];
                    f = [(pi-bearing)/pi, (pi-angle_off)/pi, (D-dist)/D];
                case 3
                    w = [0.0, 0.3, 0.7];
                    f = [bearing/pi, (pi-angle_off)/pi, dist/D];
                case 4
                    w = [0.1, 0.2, 0.7];
                    f = [(pi-bearing)/pi, (pi-angle_off)/pi, dist/D];
            end
            u = sum( w.* f);
        end
        % 动作选择 12 10
        function [best_action, utilities] = select_best_action(obj, bearing, angle_off, dist)
            posterior = obj.compute_posterior(bearing, angle_off, dist);
            num_actions = 7;
            utilities = zeros(1, num_actions);
            for a = 1:num_actions
                for s = 1:4
                    utilities(a) = utilities(a) + posterior(s) * obj.utility_given_state_action(s,bearing, angle_off, dist);
                end
            end
            [~, best_action] = max(utilities);
        end
        
    end
end