% PerformanceAnalysis.m - 性能分析类

classdef PerformanceAnalysis
    methods (Static)
        function results = analyze_channel(H, SNR_dB)
            % 分析信道性能
            cfg = Config;
            
            % 1. 计算条件数
            condition_number = cond(H);
            
            % 2. 计算奇异值
            singular_values = svd(H);
            
            % 3. 计算容量
            SNR_lin = 10^(SNR_dB/10);
            capacity = real(log2(det(eye(size(H,1)) + ...
                      (SNR_lin/size(H,2))*(H*H'))));
            
            % 4. 计算波束增益
            [U,~,V] = svd(H);
            w_rx = U(:,1);
            w_tx = V(:,1);
            beam_gain = 10*log10(abs(w_rx' * H * w_tx)^2);
            
            % 返回结果结构体
            results.condition_number = condition_number;
            results.singular_values = singular_values;
            results.capacity = capacity;
            results.beam_gain = beam_gain;
        end
        
        function plot_channel_statistics(H_history, t_vec)
            % 绘制信道统计特性
            figure('Position', [100 100 1200 400]);
            
            % 1. 信道矩阵条件数随时间变化
            subplot(1,3,1);
            cond_history = zeros(size(H_history,3), 1);
            for i = 1:size(H_history,3)
                cond_history(i) = cond(H_history(:,:,i));
            end
            semilogy(t_vec, cond_history, 'b-', 'LineWidth', 2);
            grid on;
            xlabel('时间 (s)');
            ylabel('条件数');
            title('信道矩阵条件数');
            
            % 2. 奇异值分布
            subplot(1,3,2);
            [~,S,~] = svd(H_history(:,:,end));
            semilogy(diag(S), 'ro-', 'LineWidth', 2);
            grid on;
            xlabel('索引');
            ylabel('奇异值');
            title('最终时刻奇异值分布');
            
            % 3. 信道相关性矩阵
            subplot(1,3,3);
            R = H_history(:,:,end) * H_history(:,:,end)';
            imagesc(20*log10(abs(R)));
            colorbar;
            xlabel('接收天线索引');
            ylabel('接收天线索引');
            title('空间相关性矩阵 (dB)');
        end
    end
end