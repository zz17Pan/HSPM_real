% main.m - 主程序

% 初始化
clear; clc;
rng(1);

% 配置参数
cfg = Config;

% 时间序列
t_vec = 0:cfg.Ts:1;  % 1秒仿真
num_steps = length(t_vec);

% 初始化存储结果
H_history = zeros(cfg.Nr_total, cfg.Nt_total, num_steps);
capacity_history = zeros(num_steps, 1);
angles_history = zeros(num_steps, 2);  % [theta, phi]
angles_history_los = zeros(num_steps, 2);   % LoS路径的方向角
angles_history_ref1 = zeros(num_steps, 2);  % 反射路径1的方向角
angles_history_ref2 = zeros(num_steps, 2);  % 反射路径2的方向角
distance_history = zeros(num_steps, 1);

% 分配资源
[comm_arrays, sens_arrays] = ResourceAllocation.allocate_subarrays();

% 生成波束码本
W = ResourceAllocation.generate_beamforming_codebook();

% 生成导频信号
S_pilot = ResourceAllocation.generate_pilot_signals();

% 主循环
fprintf('开始仿真...\n');
for n = 1:num_steps
    % 当前时间
    t = t_vec(n);
    
    % 更新阵列位置
    [tx_pos, rx_pos] = ArrayGeometry.initialize_array(t);
    
    % 生成信道矩阵
    H = ChannelHSPM.generate_channel(tx_pos, rx_pos, t);
    H_history(:,:,n) = H;
    
    % 计算容量
    SNR_lin = 10^(cfg.SNR_dB/10);
    capacity = real(log2(det(eye(cfg.Nr_total) + ...
               (SNR_lin/cfg.Nt_total)*(H*H'))));
    capacity_history(n) = capacity;
    
    % 计算当前位置参数
    rx_center = mean(rx_pos, 1);
    [theta, phi] = ArrayGeometry.calculate_angles(rx_center);
    angles_history(n,:) = [theta, phi];
    
    % 记录多路径角度
    [~, thetas, phis] = ChannelHSPM.generate_path_parameters(t);
    angles_history_los(n,:) = [thetas(1,1), phis(1,1)];
    angles_history_ref1(n,:) = [thetas(2,1), phis(2,1)];
    angles_history_ref2(n,:) = [thetas(3,1), phis(3,1)];
    
    % 计算距离
    distance_history(n) = norm(rx_center);
    
    % 每10ms更新一次资源分配
    if mod(n, 10) == 0
        [comm_arrays, sens_arrays] = ResourceAllocation.allocate_subarrays();
    end
    
    % 显示进度
    if mod(n, floor(num_steps/10)) == 0
        fprintf('完成 %.1f%%\n', n/num_steps*100);
    end
end

% 绘制结果
plotResults(t_vec, capacity_history, angles_history, angles_history_los, ...
           angles_history_ref1, angles_history_ref2, distance_history, cfg);

% 保存结果
save('simulation_results.mat', 'H_history', 'capacity_history', ...
     'angles_history', 'angles_history_los', 'angles_history_ref1', ...
     'angles_history_ref2', 'distance_history', 't_vec', 'cfg');

fprintf('\n===== 仿真完成 =====\n');
fprintf('平均信道容量: %.2f bits/s/Hz\n', mean(capacity_history));
fprintf('最大通信距离: %.2f m\n', max(distance_history));
fprintf('方位角范围: [%.1f°, %.1f°]\n', min(angles_history(:,1)*180/pi), max(angles_history(:,1)*180/pi));
fprintf('俯仰角范围: [%.1f°, %.1f°]\n', min(angles_history(:,2)*180/pi), max(angles_history(:,2)*180/pi));