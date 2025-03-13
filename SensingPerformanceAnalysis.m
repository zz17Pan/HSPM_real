classdef SensingPerformanceAnalysis
    methods (Static)
        function metrics = analyze_sensing_accuracy(estimated, true_values, t)
            try
                % 计算误差
                range_error = abs(estimated.range - true_values.range);
                theta_error = abs(rad2deg(estimated.theta - true_values.theta));
                phi_error = abs(rad2deg(estimated.phi - true_values.phi));
                
                % 计算统计指标
                metrics.range = struct(...
                    'error', range_error, ...
                    'rmse', sqrt(mean(range_error.^2)), ...
                    'mean', mean(range_error), ...
                    'std', std(range_error));
                
                metrics.theta = struct(...
                    'error', theta_error, ...
                    'rmse', sqrt(mean(theta_error.^2)), ...
                    'mean', mean(theta_error), ...
                    'std', std(theta_error));
                
                metrics.phi = struct(...
                    'error', phi_error, ...
                    'rmse', sqrt(mean(phi_error.^2)), ...
                    'mean', mean(phi_error), ...
                    'std', std(phi_error));
                
                % 打印性能指标
                fprintf('===== 感知性能分析 (t = %.3f s) =====\n', t);
                fprintf('距离估计:\n');
                fprintf('  RMSE: %.3f m\n', metrics.range.rmse);
                fprintf('  平均误差: %.3f m\n', metrics.range.mean);
                fprintf('  标准差: %.3f m\n', metrics.range.std);
                
                fprintf('方位角估计:\n');
                fprintf('  RMSE: %.3f°\n', metrics.theta.rmse);
                fprintf('  平均误差: %.3f°\n', metrics.theta.mean);
                fprintf('  标准差: %.3f°\n', metrics.theta.std);
                
                fprintf('俯仰角估计:\n');
                fprintf('  RMSE: %.3f°\n', metrics.phi.rmse);
                fprintf('  平均误差: %.3f°\n', metrics.phi.mean);
                fprintf('  标准差: %.3f°\n', metrics.phi.std);
                
            catch ME
                fprintf('性能分析错误:\n');
                fprintf('错误信息: %s\n', ME.message);
                fprintf('时间: %s\n', datestr(datetime('2025-03-13 06:17:19')));
                fprintf('用户: zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function visualize_sensing_performance(estimated_history, true_history, t_vec)
            try
                figure('Position', [100 100 1200 800]);
                
                % 1. 距离误差随时间变化
                subplot(3,2,1);
                plot(t_vec, abs(estimated_history.range - true_history.range), 'b-', 'LineWidth', 1.5);
                grid on;
                xlabel('时间 (s)');
                ylabel('距离误差 (m)');
                title('距离估计误差');
                
                % 2. 方位角误差随时间变化
                subplot(3,2,3);
                plot(t_vec, rad2deg(abs(estimated_history.theta - true_history.theta)), 'r-', 'LineWidth', 1.5);
                grid on;
                xlabel('时间 (s)');
                ylabel('方位角误差 (°)');
                title('方位角估计误差');
                
                % 3. 俯仰角误差随时间变化
                subplot(3,2,5);
                plot(t_vec, rad2deg(abs(estimated_history.phi - true_history.phi)), 'g-', 'LineWidth', 1.5);
                grid on;
                xlabel('时间 (s)');
                ylabel('俯仰角误差 (°)');
                title('俯仰角估计误差');
                
                % 4. 三维轨迹对比
                subplot(3,2,[2,4,6]);
                plot3(true_history.x, true_history.y, true_history.z, 'b-', 'LineWidth', 2);
                hold on;
                plot3(estimated_history.x, estimated_history.y, estimated_history.z, 'r--', 'LineWidth', 2);
                grid on;
                xlabel('X (m)');
                ylabel('Y (m)');
                zlabel('Z (m)');
                legend('真实轨迹', '估计轨迹');
                title('三维轨迹对比');
                
                % 添加时间戳
                sgtitle(sprintf('感知性能分析 (%s)', datestr(datetime('2025-03-13 06:17:19'))));
                
            catch ME
                fprintf('可视化错误:\n');
                fprintf('错误信息: %s\n', ME.message);
                fprintf('时间: %s\n', datestr(datetime('2025-03-13 06:17:19')));
                fprintf('用户: zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function metrics = calculate_cumulative_metrics(metrics_history)
            try
                % 计算累积统计指标
                metrics.range_rmse = sqrt(mean([metrics_history.range_error].^2));
                metrics.theta_rmse = sqrt(mean([metrics_history.theta_error].^2));
                metrics.phi_rmse = sqrt(mean([metrics_history.phi_error].^2));
                
                % 计算95%置信区间
                metrics.range_ci = prctile([metrics_history.range_error], [2.5 97.5]);
                metrics.theta_ci = prctile([metrics_history.theta_error], [2.5 97.5]);
                metrics.phi_ci = prctile([metrics_history.phi_error], [2.5 97.5]);
                
                % 生成性能报告
                fprintf('\n===== 累积性能报告 =====\n');
                fprintf('样本数量: %d\n', length(metrics_history));
                fprintf('距离估计:\n');
                fprintf('  RMSE: %.3f m\n', metrics.range_rmse);
                fprintf('  95%%置信区间: [%.3f, %.3f] m\n', metrics.range_ci(1), metrics.range_ci(2));
                
                fprintf('方位角估计:\n');
                fprintf('  RMSE: %.3f°\n', metrics.theta_rmse);
                fprintf('  95%%置信区间: [%.3f, %.3f]°\n', metrics.theta_ci(1), metrics.theta_ci(2));
                
                fprintf('俯仰角估计:\n');
                fprintf('  RMSE: %.3f°\n', metrics.phi_rmse);
                fprintf('  95%%置信区间: [%.3f, %.3f]°\n', metrics.phi_ci(1), metrics.phi_ci(2));
                
            catch ME
                fprintf('累积指标计算错误:\n');
                fprintf('错误信息: %s\n', ME.message);
                fprintf('时间: %s\n', datestr(datetime('2025-03-13 06:17:19')));
                fprintf('用户: zz17Pan\n');
                rethrow(ME);
            end
        end
    end
end