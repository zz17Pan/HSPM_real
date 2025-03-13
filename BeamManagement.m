% BeamManagement.m - 波束管理

classdef BeamManagement
    methods (Static)
        function [best_w_tx, best_w_rx] = beam_search(H)
            % 基于SVD的波束搜索
            [U, ~, V] = svd(H);
            best_w_rx = U(:,1);  % 最优接收波束形成向量
            best_w_tx = V(:,1);  % 最优发射波束形成向量
        end
        
        function gain = compute_beam_gain(H, w_tx, w_rx)
            % 计算波束增益
            gain = abs(w_rx' * H * w_tx)^2;
        end
    end
end