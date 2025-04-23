classdef ukf
    % 无迹卡尔曼滤波
    properties
        % 状态维度与观测维度
        n           % 状态向量维度
        m           % 观测向量维度

        % Sigma点参数
        alpha       % 控制Sigma点分布范围，表扩散程度
        beta        % 分布类型参数（高斯分布时beta=2）
        kappa       % 缩放参数，有点没有，只是方便lambda进行初始化
        lambda      % 辅助参数

        % 状态与协方差
        x           % 状态均值向量 (n×1)
        P           % 状态协方差矩阵 (n×n) 

        % 噪声协方差矩阵
        Q           % 过程噪声协方差 (n×n)
        R           % 观测噪声协方差 (m×m)

        % Sigma点权重
        weights_m   % 均值权重 (1×(2n+1))
        weights_c   % 协方差权重 (1×(2n+1))
    end

    methods
        function obj = ukf(state_dim, obs_dim, process_noise, obs_noise, alpha, beta, kappa)
            % 构造函数
            obj.n = state_dim;
            obj.m = obs_dim;

            % Sigma点参数
            obj.alpha = alpha;
            obj.beta = beta;
            obj.kappa = kappa;
            obj.lambda = alpha^2 * (state_dim + kappa) - state_dim;

            % 初始状态与协方差
            obj.x = zeros(state_dim, 1); % 状态均值，初始化为 0
            obj.P = eye(state_dim); % 协方差矩阵，初始化为单位矩阵

            % 噪声协方差矩阵
            obj.Q = process_noise;
            obj.R = obs_noise;

            % 计算权重
            obj = obj.compute_weights();
        end
         %% 计算Sigma点权重
         function obj = compute_weights(obj)
            
            obj.weights_m = zeros(1, 2*obj.n + 1);
            obj.weights_c = zeros(1, 2*obj.n + 1);

            obj.weights_m(1) = obj.lambda / (obj.n + obj.lambda);
            obj.weights_c(1) = obj.weights_m(1) + (1 - obj.alpha^2 + obj.beta);

            for i = 2:(2*obj.n + 1)
                obj.weights_m(i) = 1 / (2*(obj.n + obj.lambda));
                obj.weights_c(i) = obj.weights_m(i);
            end
        end
        %% 生成sigma点 2n+1 个
        function sigma_points = generate_sigma_points(obj)
            % 生成Sigma点矩阵 (n×(2n+1))
            sigma_points = zeros(obj.n, 2*obj.n + 1);
            sigma_points(:, 1) = obj.x;

            % Cholesky分解（添加正则化避免非正定）
            sqrt_P = chol((obj.n + obj.lambda) * (obj.P + 1e-6*eye(obj.n)), 'lower');

            % 生成Sigma点
            for i = 1:obj.n
                sigma_points(:, i+1) = obj.x + sqrt_P(:, i);
                sigma_points(:, obj.n+i+1) = obj.x - sqrt_P(:, i);
            end
        end
       %% 状态预测
        function obj = predict(obj, f_transition)
            % 预测步骤
            % 输入：f_transition - 状态转移函数句柄（输入：n×1向量，输出：n×1向量）
            sigma_points = obj.generate_sigma_points();
            
            % 传播Sigma点
            predicted_points = zeros(obj.n, 2*obj.n + 1);
            for i = 1:(2*obj.n + 1)
                predicted_points(:, i) = f_transition(sigma_points(:, i));
            end
            
            % 计算预测均值和协方差
            obj.x = predicted_points * obj.weights_m';
            obj.P = zeros(obj.n, obj.n);
            for i = 1:(2*obj.n + 1)
                diff = predicted_points(:, i) - obj.x;
                obj.P = obj.P + obj.weights_c(i) * (diff * diff');
            end
            obj.P = obj.P + obj.Q; % 加入过程噪声
        end
        %% 测量校正加滤波更新
                function obj = update(obj, z, h_observation)
            % 更新步骤
            % 输入：z - 观测值 (m×1)，h_observation - 观测函数句柄
            sigma_points = obj.generate_sigma_points();
            
            % 映射到观测空间
            obs_points = zeros(obj.m, 2*obj.n + 1);
            for i = 1:(2*obj.n + 1)
                obs_points(:, i) = h_observation(sigma_points(:, i));
            end
            
            % 计算观测均值和协方差
            z_pred = obs_points * obj.weights_m';
            P_zz = zeros(obj.m, obj.m);
            P_xz = zeros(obj.n, obj.m);
            for i = 1:(2*obj.n + 1)
                diff_z = obs_points(:, i) - z_pred;
                diff_x = sigma_points(:, i) - obj.x;
                P_zz = P_zz + obj.weights_c(i) * (diff_z * diff_z');
                P_xz = P_xz + obj.weights_c(i) * (diff_x * diff_z');
            end
            P_zz = P_zz + obj.R; % 加入观测噪声
            
            % 计算卡尔曼增益
            K = P_xz / P_zz;
            
            % 更新状态和协方差
            obj.x = obj.x + K * (z - z_pred);
            obj.P = obj.P - K * P_zz * K';
            
            % 确保协方差对称
            obj.P = (obj.P + obj.P') / 2;
        end
    end
end


