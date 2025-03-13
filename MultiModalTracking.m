classdef MultiModalTracking < handle
    properties
        sensor          % 感知处理器
        ukf             % 自适应UKF跟踪器
        kalman_filter   % 卡尔曼滤波器
        
        % 缓存数据
        true_trajectory     % 真实轨迹
        estimated_trajectory% 估计轨迹
        angle_errors       % 角度估计误差
        position_errors    % 位置估计误差
        
        % 配置参数
        cfg
        
        % 时间戳
        creation_time
        last_update_time
    end
    
    methods
        function obj = MultiModalTracking()
            try
                obj.creation_time = datetime('now', 'TimeZone', 'UTC');
                fprintf('系统初始化时间: %s\n', datestr(obj.creation_time));
                
                fprintf('初始化系统配置...\n');
                obj.cfg = Config;
                
                fprintf('初始化传感器...\n');
                obj.sensor = MultiModalSensing();
                
                fprintf('初始化UKF跟踪器...\n');
                obj.ukf = MultimodeAUKF();
                
                fprintf('初始化数据存储...\n');
                obj.initialize_storage();
                
                fprintf('系统初始化完成\n');
            catch ME
                fprintf('系统初始化失败:\n');
                fprintf('错误信息: %s\n', ME.message);
                rethrow(ME);
            end
        end
        
        function initialize_storage(obj)
            obj.true_trajectory = [];
            obj.estimated_trajectory = [];
            obj.angle_errors = [];
            obj.position_errors = [];
        end
        
        function run_tracking(obj, duration)
            try
                % 计算总帧数
                num_frames = ceil(duration/obj.cfg.Ts);
                fprintf('开始跟踪，总帧数：%d\n', num_frames);
                
                % 初始化时间
                current_time = obj.creation_time;
                
                % 主循环
                for frame = 1:num_frames
                    t = (frame-1) * obj.cfg.Ts;
                    fprintf('处理第 %d 帧 (t = %.3f s)\n', frame, t);
                    
                    % 更新时间
                    current_time = current_time + seconds(obj.cfg.Ts);
                    obj.last_update_time = current_time;
                    
                    % 1. 获取真实位置
                    true_pos = ArrayGeometry.get_rx_position(t);
                    [true_theta, true_phi] = ArrayGeometry.calculate_angles(true_pos);
                    
                    % 2. 获取天线阵列位置
                    [tx_pos, rx_pos] = ArrayGeometry.initialize_array(t);
                    fprintf('tx_pos维度: [%d, %d]\n', size(tx_pos));
                    fprintf('rx_pos维度: [%d, %d]\n', size(rx_pos));
                    
                    % 3. 多模态感知
                    sensing_result = obj.perform_multimodal_sensing(t, tx_pos, rx_pos);
                    
                    % 4. 跟踪更新
                    tracking_result = obj.update_tracking(sensing_result);
                    
                    % 5. 记录结果
                    obj.store_results(frame, true_pos, true_theta, true_phi, ...
                        tracking_result.position, tracking_result.theta, tracking_result.phi);
                    
                    % 6. 实时显示
                    if mod(frame, 10) == 0
                        obj.visualize_tracking(frame);
                    end
                end
                
            catch ME
                fprintf('跟踪过程出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('错误位置：%s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
                fprintf('最后更新时间: %s\n', datestr(obj.last_update_time));
                rethrow(ME);
            end
        end
        
        function sensing_result = perform_multimodal_sensing(obj, t, tx_pos, rx_pos)
            try
                % 1. 角度域感知
                fprintf('执行角度感知...\n');
                angle_measurement = obj.sensor.perform_angle_sensing(tx_pos, rx_pos, t);
                
                % 2. 距离-多普勒感知
                fprintf('执行距离-多普勒感知...\n');
                range_doppler = obj.sensor.perform_range_doppler_sensing(tx_pos, rx_pos);
                
                % 3. 极化域感知
                fprintf('执行极化感知...\n');
                polarization = obj.sensor.perform_polarization_sensing(tx_pos, rx_pos);
                
                % 4. 特征级融合
                fprintf('执行特征融合...\n');
                sensing_result = obj.sensor.feature_fusion(angle_measurement, range_doppler, polarization);
                
            catch ME
                fprintf('多模态感知失败：\n');
                fprintf('错误信息：%s\n', ME.message);
                if exist('tx_pos', 'var') && exist('rx_pos', 'var')
                    fprintf('tx_pos维度: [%d, %d]\n', size(tx_pos));
                    fprintf('rx_pos维度: [%d, %d]\n', size(rx_pos));
                end
                rethrow(ME);
            end
        end
        
        function result = update_tracking(obj, sensing_result)
            try
                % UKF更新
                obj.ukf.predict();
                obj.ukf.update(sensing_result);
                
                % 提取跟踪结果
                state = obj.ukf.get_state();
                result.position = state(1:3);
                result.velocity = state(4:6);
                result.theta = state(7);
                result.phi = state(8);
                
            catch ME
                fprintf('跟踪更新失败：\n');
                fprintf('错误信息：%s\n', ME.message);
                rethrow(ME);
            end
        end
        
        function store_results(obj, frame, true_pos, true_theta, true_phi, ...
                est_pos, est_theta, est_phi)
            % 存储轨迹
            obj.true_trajectory(frame,:) = true_pos;
            obj.estimated_trajectory(frame,:) = est_pos;
            
            % 计算误差
            angle_error = [abs(true_theta - est_theta), abs(true_phi - est_phi)];
            position_error = norm(true_pos - est_pos);
            
            obj.angle_errors(frame,:) = angle_error;
            obj.position_errors(frame) = position_error;
        end
        
        function visualize_tracking(obj, frame)
            figure(1); clf;
            
            % 1. 3D轨迹显示
            subplot(2,2,1);
            plot3(obj.true_trajectory(1:frame,1), ...
                  obj.true_trajectory(1:frame,2), ...
                  obj.true_trajectory(1:frame,3), 'b-', 'LineWidth', 2);
            hold on;
            plot3(obj.estimated_trajectory(1:frame,1), ...
                  obj.estimated_trajectory(1:frame,2), ...
                  obj.estimated_trajectory(1:frame,3), 'r--', 'LineWidth', 2);
            grid on;
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            legend('真实轨迹', '估计轨迹');
            title('3D轨迹对比');
            
            % 2. 角度误差
            subplot(2,2,2);
            plot(obj.angle_errors(:,1)*180/pi, 'b-', 'LineWidth', 1.5);
            hold on;
            plot(obj.angle_errors(:,2)*180/pi, 'r-', 'LineWidth', 1.5);
            grid on;
            xlabel('帧数'); ylabel('误差 (度)');
            legend('方位角误差', '俯仰角误差');
            title('角度估计误差');
            
            % 3. 位置误差
            subplot(2,2,3);
            plot(obj.position_errors, 'k-', 'LineWidth', 1.5);
            grid on;
            xlabel('帧数'); ylabel('误差 (m)');
            title('位置估计误差');
            
            % 4. 性能统计
            subplot(2,2,4);
            text(0.1, 0.8, sprintf('平均角度误差: %.2f°', mean(obj.angle_errors(:))*180/pi));
            text(0.1, 0.6, sprintf('平均位置误差: %.2f m', mean(obj.position_errors)));
            text(0.1, 0.4, sprintf('最大角度误差: %.2f°', max(obj.angle_errors(:))*180/pi));
            text(0.1, 0.2, sprintf('最大位置误差: %.2f m', max(obj.position_errors)));
            axis off;
            title('性能统计');
            
            drawnow;
        end
        
        function display_final_results(obj)
            % 计算统计指标
            mean_pos_error = mean(obj.position_errors);
            std_pos_error = std(obj.position_errors);
            mean_angle_error = mean(obj.angle_errors);
            std_angle_error = std(obj.angle_errors);
            
            % 显示结果
            fprintf('\n=== 跟踪性能分析 ===\n');
            fprintf('系统运行时间: %s 至 %s\n', ...
                    datestr(obj.creation_time), datestr(obj.last_update_time));
            fprintf('位置估计:\n');
            fprintf('  平均误差: %.2f m\n', mean_pos_error);
            fprintf('  标准差: %.2f m\n', std_pos_error);
            fprintf('角度估计:\n');
            fprintf('  平均方位角误差: %.2f°\n', mean_angle_error(1)*180/pi);
            fprintf('  平均俯仰角误差: %.2f°\n', mean_angle_error(2)*180/pi);
            fprintf('  方位角标准差: %.2f°\n', std_angle_error(1)*180/pi);
            fprintf('  俯仰角标准差: %.2f°\n', std_angle_error(2)*180/pi);
        end
    end
end