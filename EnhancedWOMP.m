classdef EnhancedWOMP < handle
    properties
        K_max           % 最大迭代次数
        alpha           % 加权因子
        tolerance       % 误差容限
    end
    
    methods
        function obj = EnhancedWOMP(N_ant)
            obj.K_max = ceil(0.2 * N_ant);
            obj.alpha = 0.7;
            obj.tolerance = 1e-6;
        end
        
        function [idx_set, x_hat] = process(obj, y, Phi, rho)
            if nargin < 4
                rho = ones(size(Phi, 2), 1);
            end
            
            residual = y;
            idx_set = [];
            x_hat = [];
            
            for k = 1:obj.K_max
                % 多模态加权投影
                projection = abs(Phi' * residual) + obj.alpha * rho;
                [~, idx] = max(projection);
                idx_set = [idx_set; idx];
                
                % 子空间正交化
                Phi_sub = Phi(:, idx_set);
                x_hat = pinv(Phi_sub) * y;
                residual = y - Phi_sub * x_hat;
                
                % 检查收敛
                if norm(residual) < obj.tolerance
                    break;
                end
                
                % 动态权重衰减
                obj.alpha = obj.alpha * 0.9;
            end
        end
    end
end