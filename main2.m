% main.m - 混合感知系统主程序
% 作者: zz17Pan
% 更新时间: 2025-03-13 06:19:42

%% 初始化
clear; clc; close all;
rng(1);  % 设置随机数种子，确保结果可重复

try
    % 记录开始时间
    start_time = datetime('2025-03-13 06:19:42', 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
    fprintf('程序开始执行时间: %s\n', datestr(start_time));
    fprintf('用户: zz17Pan\n\n');

    %% 配置参数
    cfg = Config;
    
    % 时间序列设置
    t_vec = 0:cfg.Ts:1;  % 1秒仿真
    num_steps = length(t_vec);
    
    %% 初始化存储结构
    % 信道信息存储
    H_history = zeros(cfg.Nr_total, cfg.Nt_total, num_steps);
    capacity_history = zeros(num_steps, 1);
    
    % 真实轨迹信息
    true_history = struct(...
        'range', zeros(num_steps, 1), ...
        'theta', zeros(num_steps, 1), ...
        'phi', zeros(num_steps, 1), ...
        'x', zeros(num_steps, 1), ...
        'y', zeros(num_steps, 1), ...
        'z', zeros(num_steps, 1));
    
    % 估计轨迹信息
    estimated_history = struct(...
        'range', zeros(num_steps, 1), ...
        'theta', zeros(num_steps, 1), ...
        'phi', zeros(num_steps, 1), ...
        'x', zeros(num_steps, 1), ...
        'y', zeros(num_steps, 1), ...
        'z', zeros(num_steps, 1));
    
    % 性能评估指标
    sensing_metrics = struct(...
        'time', zeros(num_steps, 1), ...
        'range_error', zeros(num_steps, 1), ...
        'theta_error', zeros(num_steps, 1), ...
        'phi_error', zeros(num_steps, 1));

    %% 初始化系统组件
    fprintf('初始化系统组件...\n');
    
    % 初始化感知处理器
    sensor = MultiModalSensing();
    
    % 初始化跟踪器
    tracker = MultiModalTracking();
    
    % 初始化资源分配
    [comm_arrays, sens_arrays] = ResourceAllocation.allocate_subarrays();
    
    % 生成导频信号
    S_pilot = ResourceAllocation.generate_pilot_signals();
    
    fprintf('系统初始化完成\n\n');

    %% 主循环
    fprintf('开始仿真...\n');
    for n = 1:num_steps
        % 当前时间
        t = t_vec(n);
        fprintf('\n处理时间步 %d/%d (t = %.3f s)\n', n, num_steps, t);
        
        % 1. 更新阵列位置
        [tx_pos, rx_pos] = ArrayGeometry.initialize_array(t);
        rx_center = mean(rx_pos, 1);
        
        % 2. 获取真实参数
        [true_theta, true_phi] = ArrayGeometry.calculate_angles(rx_center);
        true_range = norm(rx_center);
        
        % 3. 生成信道矩阵
        H = ChannelHSPM.generate_channel(tx_pos, rx_pos, t);
        H_history(:,:,n) = H;
        
        % 4. 执行多模态感知
        sensing_result = sensor.perform_multimodal_sensing(tx_pos, rx_pos, t);
        
        % 5. 更新跟踪器
        tracking_result = tracker.update_tracking(sensing_result);
        
        % 6. 记录真实轨迹
        true_history.range(n) = true_range;
        true_history.theta(n) = true_theta;
        true_history.phi(n) = true_phi;
        true_history.x(n) = rx_center(1);
        true_history.y(n) = rx_center(2);
        true_history.z(n) = rx_center(3);
        
        % 7. 记录估计轨迹
        estimated_history.range(n) = sensing_result.range;
        estimated_history.theta(n) = sensing_result.theta;
        estimated_history.phi(n) = sensing_result.phi;
        estimated_history.x(n) = tracking_result.position(1);
        estimated_history.y(n) = tracking_result.position(2);
        estimated_history.z(n) = tracking_result.position(3);
        
        % 8. 计算性能指标
        metrics = SensingPerformanceAnalysis.analyze_sensing_accuracy(...
            sensing_result, ...
            struct('range', true_range, 'theta', true_theta, 'phi', true_phi), ...
            t);
        
        % 9. 存储性能指标
        sensing_metrics.time(n) = t;
        sensing_metrics.range_error(n) = metrics.range.error;
        sensing_metrics.theta_error(n) = metrics.theta.error;
        sensing_metrics.phi_error(n) = metrics.phi.error;
        
        % 10. 计算信道容量
        SNR_lin = 10^(cfg.SNR_dB/10);
        capacity = real(log2(det(eye(cfg.Nr_total) + ...
                   (SNR_lin/cfg.Nt_total)*(H*H'))));
        capacity_history(n) = capacity;
        
        % 11. 每10步显示一次进度
        if mod(n, 10) == 0
            fprintf('完成 %.1f%%\n', n/num_steps*100);
            % 实时绘制跟踪结果
            tracker.visualize_tracking(n);
            drawnow;
        end
        
        % 12. 动态资源分配（每10ms更新一次）
        if mod(n, 10) == 0
            [comm_arrays, sens_arrays] = ResourceAllocation.allocate_subarrays();
        end
    end

    %% 性能评估与可视化
    fprintf('\n开始性能评估...\n');
    
    % 1. 显示感知性能
    SensingPerformanceAnalysis.visualize_sensing_performance(...
        estimated_history, true_history, t_vec);
    
    % 2. 计算累积性能指标
    cumulative_metrics = SensingPerformanceAnalysis.calculate_cumulative_metrics(sensing_metrics);
    
    % 3. 分析信道性能
    channel_performance = PerformanceAnalysis.analyze_channel(H_history(:,:,end), cfg.SNR_dB);
    PerformanceAnalysis.plot_channel_statistics(H_history, t_vec);
    
    % 4. 显示跟踪器最终结果
    tracker.display_final_results();

    %% 保存结果
    save_results = true;  % 设置为false可以跳过保存
    if save_results
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        save(['results_', timestamp, '.mat'], ...
            'H_history', 'capacity_history', ...
            'true_history', 'estimated_history', ...
            'sensing_metrics', 'cumulative_metrics', ...
            'channel_performance', 'cfg', 't_vec');
        fprintf('\n结果已保存至 results_%s.mat\n', timestamp);
    end

    %% 程序结束
    end_time = datetime('now', 'TimeZone', 'UTC');
    fprintf('\n程序执行完成\n');
    fprintf('开始时间: %s\n', datestr(start_time));
    fprintf('结束时间: %s\n', datestr(end_time));
    fprintf('总运行时间: %s\n', char(end_time - start_time));
    fprintf('用户: zz17Pan\n');

catch ME
    % 错误处理
    fprintf('\n程序执行出错:\n');
    fprintf('错误信息: %s\n', ME.message);
    fprintf('错误位置: %s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
    fprintf('时间: %s\n', datestr(datetime('2025-03-13 06:19:42')));
    fprintf('用户: zz17Pan\n');
    
    % 保存错误日志
    log_filename = ['error_log_', datestr(now, 'yyyymmdd_HHMMSS'), '.mat'];
    save(log_filename, 'ME');
    fprintf('错误日志已保存至 %s\n', log_filename);
    
    % 重新抛出错误
    rethrow(ME);
end