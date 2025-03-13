classdef MultiModalSensing < handle
    properties
        womp            % 加权正交匹配追踪
        dictionary      % 角度-距离字典
        cfg             % 配置参数
        
        % FMCW参数
        chirp_period    % chirp周期
        bandwidth      % 带宽
        num_range_bins % 距离单元数
        num_doppler_bins% 多普勒单元数
        
        % 极化参数
        pol_matrix     % 极化矩阵
        gamma          % 极化角度
        eta            % 极化相位
        
        % 调试信息
        last_error_time
        error_count
        creation_time
    end
    
    methods
        function obj = MultiModalSensing()
            try
                obj.creation_time = datetime('2025-03-13 05:30:40', 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
                fprintf('MultiModalSensing初始化时间: %s\n', datestr(obj.creation_time));
                fprintf('用户: zz17Pan\n');
                
                obj.cfg = Config;
                fprintf('系统参数:\n');
                fprintf('Nx = %d, Nz = %d\n', obj.cfg.Nx, obj.cfg.Nz);
                fprintf('每个子阵天线总数 = %d\n', obj.cfg.Nx * obj.cfg.Nz);
                
                obj.initialize_parameters();
                
                fprintf('正在生成联合字典...\n');
                obj.initialize_dictionary();
                obj.womp = EnhancedWOMP(obj.cfg.Nx * obj.cfg.Nz);
                
                obj.error_count = 0;
                obj.last_error_time = obj.creation_time;
                
                fprintf('MultiModalSensing初始化完成\n');
            catch ME
                fprintf('MultiModalSensing初始化失败：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:30:40', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function initialize_parameters(obj)
            obj.chirp_period = 1e-3;  % 1ms
            obj.bandwidth = obj.cfg.B; % 2GHz
            obj.num_range_bins = 256;
            obj.num_doppler_bins = 64;
            
            obj.gamma = pi/4;  % 初始极化角度
            obj.eta = 0;       % 初始极化相位
            obj.pol_matrix = obj.generate_polarization_matrix();
        end
        
        function initialize_dictionary(obj)
            obj.dictionary = obj.generate_joint_dictionary();
        end
        
        function dict = generate_joint_dictionary(obj)
            try
                % 角度网格定义
                theta_grid = linspace(-pi, pi, 180);        % 方位角
                phi_grid = linspace(-pi/2, pi/2, 90);       % 俯仰角
                r_grid = linspace(0, obj.cfg.D0*1.5, 100);  % 距离网格
                
                % 初始化字典
                N_theta = length(theta_grid);
                N_phi = length(phi_grid);
                N_r = length(r_grid);
                N_atoms = N_theta * N_phi * N_r;
                N_ant = obj.cfg.Nx * obj.cfg.Nz;  % 每个子阵的天线数
                
                fprintf('生成字典，维度: %d x %d\n', N_ant, N_atoms);
                dict = zeros(N_ant, N_atoms);
                
                % 生成字典原子
                idx = 1;
                for i_theta = 1:N_theta
                    for i_phi = 1:N_phi
                        for i_r = 1:N_r
                            % 获取当前参数
                            theta = theta_grid(i_theta);
                            phi = phi_grid(i_phi);
                            r = r_grid(i_r);
                            
                            % 生成方向矢量
                            a = ChannelHSPM.steering_vector_2d(theta, phi, true);
                            
                            % 检查维度
                            if length(a) ~= N_ant
                                error('方向矢量维度不匹配：预期 %d，实际 %d', N_ant, length(a));
                            end
                            
                            % 添加距离信息
                            phase_r = exp(-1j*2*pi*r/obj.cfg.lambda_sens);
                            dict(:,idx) = a * phase_r;
                            
                            idx = idx + 1;
                        end
                    end
                end
                
                % 归一化字典
                for i = 1:N_atoms
                    dict(:,i) = dict(:,i) / norm(dict(:,i));
                end
                
                fprintf('字典生成完成\n');
                
            catch ME
                fprintf('字典生成失败：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:30:40', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function result = perform_angle_sensing(obj, tx_pos, rx_pos, t)
            try
                % 获取感知子阵的索引
                [~, sens_subarrays] = ResourceAllocation.allocate_subarrays();
                
                % 计算子阵索引范围
                tx_start = (sens_subarrays.tx-1)*obj.cfg.N_at + 1;
                tx_end = min(tx_start + obj.cfg.N_at - 1, size(tx_pos,1));
                rx_start = (sens_subarrays.rx-1)*obj.cfg.N_ar + 1;
                rx_end = min(rx_start + obj.cfg.N_ar - 1, size(rx_pos,1));
                
                % 提取有效的发射和接收位置
                valid_tx_idx = tx_start:tx_end;
                valid_rx_idx = rx_start:rx_end;
                
                % 打印调试信息
                fprintf('角度感知子阵索引：\n');
                fprintf('tx索引范围: [%d, %d]\n', min(valid_tx_idx), max(valid_tx_idx));
                fprintf('rx索引范围: [%d, %d]\n', min(valid_rx_idx), max(valid_rx_idx));
                
                % 生成导频信号
                s_pilot = ResourceAllocation.generate_pilot_signals(length(valid_tx_idx));
                
                % 生成感知信道
                H_sens = ChannelHSPM.generate_channel(tx_pos(valid_tx_idx,:), ...
                                                    rx_pos(valid_rx_idx,:), t);
                
                % 添加噪声
                noise = (randn(size(H_sens,1),size(s_pilot,2)) + ...
                        1j*randn(size(H_sens,1),size(s_pilot,2)))/sqrt(2);
                SNR_linear = 10^(obj.cfg.SNR_dB/10);
                y_sens = H_sens * s_pilot + noise/sqrt(SNR_linear);
                
                % WOMP估计
                prior = obj.generate_prior();
                [idx_set, x_hat] = obj.womp.process(y_sens(:,1), obj.dictionary, prior);
                
                % 从支持集重建参数
                result = obj.reconstruct_parameters(idx_set, x_hat);
                
            catch ME
                obj.error_count = obj.error_count + 1;
                obj.last_error_time = datetime('2025-03-13 05:30:40', 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
                fprintf('角度感知过程出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('错误位置：%s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
                fprintf('时间：%s\n', datestr(obj.last_error_time));
                fprintf('用户：zz17Pan\n');
                if exist('H_sens', 'var')
                    fprintf('H_sens维度: [%d, %d]\n', size(H_sens));
                end
                if exist('s_pilot', 'var')
                    fprintf('s_pilot维度: [%d, %d]\n', size(s_pilot));
                end
                rethrow(ME);
            end
        end
        
        function result = perform_range_doppler_sensing(obj, tx_pos, rx_pos)
            try
                % 生成FMCW信号
                t = 0:1/obj.cfg.B:obj.chirp_period-1/obj.cfg.B;
                s_fmcw = exp(1j*2*pi*(obj.cfg.f_sens*t + ...
                        obj.bandwidth/(2*obj.chirp_period)*t.^2));
                
                % 获取感知子阵的索引
                [~, sens_subarrays] = ResourceAllocation.allocate_subarrays();
                
                % 计算子阵索引范围
                tx_start = (sens_subarrays.tx-1)*obj.cfg.N_at + 1;
                tx_end = min(tx_start + obj.cfg.N_at - 1, size(tx_pos,1));
                rx_start = (sens_subarrays.rx-1)*obj.cfg.N_ar + 1;
                rx_end = min(rx_start + obj.cfg.N_ar - 1, size(rx_pos,1));
                
                % 提取有效的发射和接收位置
                valid_tx_idx = tx_start:tx_end;
                valid_rx_idx = rx_start:rx_end;
                
                % 打印调试信息
                fprintf('距离多普勒感知子阵索引：\n');
                fprintf('tx索引范围: [%d, %d]\n', min(valid_tx_idx), max(valid_tx_idx));
                fprintf('rx索引范围: [%d, %d]\n', min(valid_rx_idx), max(valid_rx_idx));
                
                % 验证索引范围
                if max(valid_tx_idx) > size(tx_pos,1) || max(valid_rx_idx) > size(rx_pos,1)
                    error('子阵索引超出范围：tx=[%d,%d]/%d, rx=[%d,%d]/%d', ...
                          min(valid_tx_idx), max(valid_tx_idx), size(tx_pos,1), ...
                          min(valid_rx_idx), max(valid_rx_idx), size(rx_pos,1));
                end
                
                % 生成接收信号
                H_sens = ChannelHSPM.generate_channel(tx_pos(valid_tx_idx,:), ...
                                                    rx_pos(valid_rx_idx,:), 0);
                rx_signal = H_sens * s_fmcw.';
                
                % 二维FFT处理
                [range_profile, doppler_profile] = obj.process_fmcw(rx_signal);
                
                % 参数估计
                result = obj.estimate_range_doppler(range_profile, doppler_profile);
                
            catch ME
                obj.error_count = obj.error_count + 1;
                obj.last_error_time = datetime('2025-03-13 05:30:40', 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
                fprintf('距离多普勒感知过程出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('错误位置：%s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
                fprintf('时间：%s\n', datestr(obj.last_error_time));
                fprintf('用户：zz17Pan\n');
                if exist('tx_pos', 'var') && exist('rx_pos', 'var')
                    fprintf('tx_pos维度: [%d, %d]\n', size(tx_pos));
                    fprintf('rx_pos维度: [%d, %d]\n', size(rx_pos));
                end
                rethrow(ME);
            end
        end
        
        function result = perform_polarization_sensing(obj, tx_pos, rx_pos)
            try
                % 获取感知子阵的索引
                [~, sens_subarrays] = ResourceAllocation.allocate_subarrays();
                
                % 计算子阵索引范围
                tx_start = (sens_subarrays.tx-1)*obj.cfg.N_at + 1;
                tx_end = min(tx_start + obj.cfg.N_at - 1, size(tx_pos,1));
                rx_start = (sens_subarrays.rx-1)*obj.cfg.N_ar + 1;
                rx_end = min(rx_start + obj.cfg.N_ar - 1, size(rx_pos,1));
                
                % 提取有效的发射和接收位置
                valid_tx_idx = tx_start:tx_end;
                valid_rx_idx = rx_start:rx_end;
                
                % 打印调试信息
                fprintf('极化感知子阵索引：\n');
                fprintf('tx索引范围: [%d, %d]\n', min(valid_tx_idx), max(valid_tx_idx));
                fprintf('rx索引范围: [%d, %d]\n', min(valid_rx_idx), max(valid_rx_idx));
                
                % 生成双极化发射信号
                s_pol = [1; 1]/sqrt(2);
                
                % 生成极化信道
                H_sens = ChannelHSPM.generate_channel(tx_pos(valid_tx_idx,:), ...
                                                    rx_pos(valid_rx_idx,:), 0);
                H_pol = obj.pol_matrix * H_sens;
                
                % 接收信号处理
                rx_pol = H_pol * s_pol;
                
                % 极化参数估计
                result = obj.estimate_polarization(rx_pol);
                
            catch ME
                obj.error_count = obj.error_count + 1;
                obj.last_error_time = datetime('2025-03-13 05:30:40', 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
                fprintf('极化感知过程出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('错误位置：%s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
                fprintf('时间：%s\n', datestr(obj.last_error_time));
                fprintf('用户：zz17Pan\n');
                if exist('H_sens', 'var')
                    fprintf('H_sens维度: [%d, %d]\n', size(H_sens));
                end
                rethrow(ME);
            end
        end
        
        function result = feature_fusion(obj, angle_meas, range_meas, pol_meas)
            try
                % 计算各测量的置信度
                m_angle = obj.calculate_confidence(angle_meas);
                m_range = obj.calculate_confidence(range_meas);
                m_pol = obj.calculate_confidence(pol_meas);
                
                % 打印调试信息
                fprintf('特征融合置信度：\n');
                fprintf('角度测量置信度: %.4f\n', m_angle);
                fprintf('距离测量置信度: %.4f\n', m_range);
                fprintf('极化测量置信度: %.4f\n', m_pol);
                
                % D-S证据理论融合
                result = obj.ds_fusion([m_angle, m_range, m_pol]);
                
            catch ME
                obj.error_count = obj.error_count + 1;
                obj.last_error_time = datetime('2025-03-13 05:34:21', 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
                fprintf('特征融合过程出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('错误位置：%s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
                fprintf('时间：%s\n', datestr(obj.last_error_time));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function prior = generate_prior(obj)
            prior = ones(size(obj.dictionary, 2), 1);  % 均匀先验
            prior = prior / sum(prior);  % 归一化
        end
        
        function result = reconstruct_parameters(obj, idx_set, x_hat)
            try
                N_theta = 180;
                N_phi = 90;
                N_r = 100;
                
                % 计算索引对应的参数
                [i_r, i_phi, i_theta] = ind2sub([N_r, N_phi, N_theta], idx_set(1));
                
                % 转换为物理参数
                theta = linspace(-pi, pi, N_theta);
                phi = linspace(-pi/2, pi/2, N_phi);
                r = linspace(0, obj.cfg.D0*1.5, N_r);
                
                result.theta = theta(i_theta);
                result.phi = phi(i_phi);
                result.range = r(i_r);
                result.amplitude = abs(x_hat(1));
                result.phase = angle(x_hat(1));
                
                % 打印调试信息
                fprintf('参数重建结果：\n');
                fprintf('方位角: %.2f°\n', result.theta*180/pi);
                fprintf('俯仰角: %.2f°\n', result.phi*180/pi);
                fprintf('距离: %.2f m\n', result.range);
                fprintf('幅度: %.4f\n', result.amplitude);
                fprintf('相位: %.2f°\n', result.phase*180/pi);
                
            catch ME
                fprintf('参数重建过程出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:34:21', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function [range_profile, doppler_profile] = process_fmcw(obj, rx_signal)
            try
                % 距离FFT
                range_profile = fft(rx_signal, obj.num_range_bins);
                
                % 多普勒FFT
                doppler_profile = fft(range_profile, obj.num_doppler_bins, 2);
                
                % 打印调试信息
                fprintf('FMCW信号处理：\n');
                fprintf('距离剖面维度: [%d, %d]\n', size(range_profile));
                fprintf('多普勒剖面维度: [%d, %d]\n', size(doppler_profile));
                
            catch ME
                fprintf('FMCW信号处理出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:34:21', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function result = estimate_range_doppler(obj, range_profile, doppler_profile)
            try
                % 找到峰值
                [~, max_idx] = max(abs(doppler_profile(:)));
                [r_idx, d_idx] = ind2sub(size(doppler_profile), max_idx);
                
                % 转换为物理量
                result.range = (r_idx-1) * obj.cfg.c/(2*obj.bandwidth);
                result.velocity = (d_idx-obj.num_doppler_bins/2) * ...
                                obj.cfg.lambda_sens/(2*obj.chirp_period);
                                
                % 打印调试信息
                fprintf('距离多普勒估计结果：\n');
                fprintf('距离: %.2f m\n', result.range);
                fprintf('速度: %.2f m/s\n', result.velocity);
                
            catch ME
                fprintf('距离多普勒估计出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:34:21', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function J = generate_polarization_matrix(obj)
            J = [cos(obj.gamma), sin(obj.gamma)*exp(1j*obj.eta);
                 -sin(obj.gamma)*exp(-1j*obj.eta), cos(obj.gamma)];
        end
        
        function result = estimate_polarization(obj, rx_pol)
            try
                pol_ratio = rx_pol(2,:) ./ rx_pol(1,:);
                gamma_est = atan(abs(mean(pol_ratio)));
                eta_est = angle(mean(pol_ratio));
                
                result.gamma = gamma_est;
                result.eta = eta_est;
                
                % 打印调试信息
                fprintf('极化估计结果：\n');
                fprintf('极化角: %.2f°\n', gamma_est*180/pi);
                fprintf('极化相位: %.2f°\n', eta_est*180/pi);
                
            catch ME
                fprintf('极化估计出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:34:21', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function conf = calculate_confidence(obj, meas)
            try
                SNR = 10^(obj.cfg.SNR_dB/10);
                conf = 1 - exp(-SNR/10);
                
                % 打印调试信息
                fprintf('置信度计算：\n');
                fprintf('SNR: %.2f dB\n', obj.cfg.SNR_dB);
                fprintf('置信度: %.4f\n', conf);
                
            catch ME
                fprintf('置信度计算出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:34:21', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function result = ds_fusion(obj, m)
            try
                K = prod(m);
                result = K / sum(K);
                
                % 打印调试信息
                fprintf('D-S融合结果：%.4f\n', result);
                
            catch ME
                fprintf('D-S融合出错：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:34:21', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
    end
end