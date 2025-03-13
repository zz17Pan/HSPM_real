classdef MultimodeAUKF < handle
    properties
        x_est           % 状态估计
        P_est           % 协方差矩阵
        Q               % 过程噪声协方差
        R               % 测量噪声协方差
        alpha          % UKF参数
        beta           % UKF参数
        kappa          % UKF参数
        model_prob     % 模型概率
        Ts             % 采样周期
    end
    
    methods
        function obj = MultimodeAUKF()
            % 初始化UKF参数
            obj.alpha = 0.001;
            obj.beta = 2;
            obj.kappa = 0;
            obj.Ts = 0.001;
            
            % 初始化状态向量和协方差
            obj.x_est = zeros(12, 1);  % [位置, 速度, 加速度, 角度]
            obj.P_est = eye(12);
            
            % 设置噪声协方差
            obj.Q = diag([0.1*ones(1,3), 0.01*ones(1,3), 0.001*ones(1,6)]);
            obj.R = diag([0.1, 0.1, 0.1, 0.01, 0.01]);
            
            % 初始化模型概率
            obj.model_prob = [0.7, 0.2, 0.1];  % CV, CA, CT模型
        end
        
        function predict(obj)
            % 状态预测
            for i = 1:3  % 三个运动模型
                [obj.x_est(:,i), obj.P_est(:,:,i)] = ...
                    obj.model_predict(obj.x_est, obj.P_est, i);
            end
        end
        
        function update(obj, z)
            % 测量更新
            for i = 1:3
                % 计算Kalman增益
                K = obj.calculate_gain(obj.P_est(:,:,i), obj.R);
                
                % 更新状态和协方差
                obj.x_est(:,i) = obj.x_est(:,i) + K*(z - obj.h(obj.x_est(:,i)));
                obj.P_est(:,:,i) = (eye(12) - K*obj.H())*obj.P_est(:,:,i);
                
                % 更新模型概率
                obj.model_prob(i) = obj.calculate_model_likelihood(z, i);
            end
            
            % 融合多模型估计
            obj.fuse_estimates();
        end
        
        function x = get_state(obj)
            % 获取当前状态估计
            x = obj.x_est;
        end
    end
    
    methods (Access = private)
        function [x_pred, P_pred] = model_predict(obj, x, P, model_idx)
            % 根据不同运动模型进行预测
            F = obj.get_transition_matrix(model_idx);
            x_pred = F * x;
            P_pred = F * P * F' + obj.Q;
        end
        
        function F = get_transition_matrix(obj, model_idx)
            % 获取状态转移矩阵
            switch model_idx
                case 1  % CV模型
                    F = [1 obj.Ts 0; 0 1 0; 0 0 0];
                case 2  % CA模型
                    F = [1 obj.Ts obj.Ts^2/2; 0 1 obj.Ts; 0 0 1];
                case 3  % CT模型
                    omega = 0.1;  % 转弯角速度
                    F = [1 sin(omega*obj.Ts)/omega (1-cos(omega*obj.Ts))/omega;
                         0 cos(omega*obj.Ts) sin(omega*obj.Ts);
                         0 -sin(omega*obj.Ts) cos(omega*obj.Ts)];
            end
            F = blkdiag(F, F, F, eye(3));  % 扩展到完整状态空间
        end
        
        function K = calculate_gain(obj, P, R)
            % 计算Kalman增益
            H = obj.H();
            K = P * H' / (H * P * H' + R);
        end
        
        function H = H(obj)
            % 测量矩阵
            H = [eye(3), zeros(3,9);   % 位置测量
                 zeros(2,6), eye(2), zeros(2,4)];  % 角度测量
        end
        
        function z_pred = h(obj, x)
            % 测量方程
            z_pred = obj.H() * x;
        end
        
        function L = calculate_model_likelihood(obj, z, model_idx)
            % 计算模型似然度
            z_pred = obj.h(obj.x_est(:,model_idx));
            v = z - z_pred;
            S = obj.H() * obj.P_est(:,:,model_idx) * obj.H()' + obj.R;
            L = (1/sqrt(det(2*pi*S))) * exp(-0.5*v'*inv(S)*v);
        end
        
        function fuse_estimates(obj)
            % 融合多模型估计结果
            x_fused = zeros(12,1);
            P_fused = zeros(12,12);
            
            % 归一化模型概率
            obj.model_prob = obj.model_prob / sum(obj.model_prob);
            
            % 加权融合
            for i = 1:3
                x_fused = x_fused + obj.model_prob(i) * obj.x_est(:,i);
                P_fused = P_fused + obj.model_prob(i) * ...
                    (obj.P_est(:,:,i) + (obj.x_est(:,i) - x_fused) * ...
                    (obj.x_est(:,i) - x_fused)');
            end
            
            obj.x_est = x_fused;
            obj.P_est = P_fused;
        end
    end
end