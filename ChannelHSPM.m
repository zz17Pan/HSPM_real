% ChannelHSPM.m - 混合球面传播模型

classdef ChannelHSPM
    methods (Static)
        function H = generate_channel(tx_pos, rx_pos, t)
            try
                cfg = Config;
                
                % 验证输入维度
                [Nt, dim_t] = size(tx_pos);
                [Nr, dim_r] = size(rx_pos);
                
                if dim_t ~= 3 || dim_r ~= 3
                    error('位置矩阵维度错误：需要Nx3矩阵，得到tx[%d,%d], rx[%d,%d]', ...
                        Nt, dim_t, Nr, dim_r);
                end
                
                % 初始化信道矩阵
                H = zeros(Nr, Nt);
                
                % 生成多径参数
                [alphas, thetas, phis] = ChannelHSPM.generate_path_parameters(t);
                
                % 处理每条路径
                for p = 1:3
                    H_path = zeros(Nr, Nt);
                    
                    % 处理每对子阵
                    for kr = 1:cfg.Kr
                        for kt = 1:cfg.Kt
                            % 判断是否为感知子阵
                            is_sensing = (kt == cfg.Kt) || (kr == cfg.Kr);
                            
                            % 计算子阵大小
                            N_at = cfg.N_at;
                            N_ar = cfg.N_ar;
                            
                            % 获取子阵索引
                            tx_start = (kt-1)*N_at + 1;
                            rx_start = (kr-1)*N_ar + 1;
                            
                            % 验证索引范围
                            if tx_start + N_at - 1 > Nt || rx_start + N_ar - 1 > Nr
                                error('子阵索引超出范围: tx=[%d,%d]/%d, rx=[%d,%d]/%d', ...
                                    tx_start, tx_start+N_at-1, Nt, ...
                                    rx_start, rx_start+N_ar-1, Nr);
                            end
                            
                            % 获取有效的索引范围
                            tx_idx = tx_start:min(tx_start+N_at-1, Nt);
                            rx_idx = rx_start:min(rx_start+N_ar-1, Nr);
                            
                            % 获取当前子阵块的天线位置
                            tx_block = tx_pos(tx_idx, :);
                            rx_block = rx_pos(rx_idx, :);
                            
                            % 计算子阵中心
                            tx_center = mean(tx_block, 1);
                            rx_center = mean(rx_block, 1);
                            
                            % 计算子阵间距离
                            r_kt_kr = norm(rx_center - tx_center);
                            
                            % 计算路径损耗因子
                            L = ChannelHSPM.path_loss(r_kt_kr, is_sensing);
                            
                            % 获取当前路径的方向角
                            theta_t = thetas(p,1); phi_t = phis(p,1);
                            theta_r = thetas(p,2); phi_r = phis(p,2);
                            
                            % 生成发射和接收方向矢量
                            a_tx = ChannelHSPM.steering_vector_2d(theta_t, phi_t, is_sensing);
                            a_rx = ChannelHSPM.steering_vector_2d(theta_r, phi_r, is_sensing);
                            
                            % 构造子阵信道块
                            block = L * (a_rx(1:length(rx_idx)) * a_tx(1:length(tx_idx))');
                            
                            % 添加到路径矩阵
                            H_path(rx_idx, tx_idx) = alphas(p) * block;
                        end
                    end
                    
                    % 累加路径贡献
                    H = H + H_path;
                end
                
                % 应用功率缩放因子
                H = cfg.Beta * H;
                
                % 打印调试信息
                fprintf('信道矩阵生成完成 (t = %.3f s):\n', t);
                fprintf('输入维度: tx[%d,%d], rx[%d,%d]\n', Nt, dim_t, Nr, dim_r);
                fprintf('输出维度: H[%d,%d]\n', size(H,1), size(H,2));
                
            catch ME
                fprintf('信道生成错误：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('错误位置：%s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 07:11:32')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        % 其他方法保持不变
        function [alphas, thetas, phis] = generate_path_parameters(t)
            % 路径复数增益
            alphas = [(1+0j), ... % LoS路径
                     (0.7+0.7j)/sqrt(2), ... % 第一反射路径
                     (0.5+0.5j)/sqrt(2)]';   % 第二反射路径
            
            % 获取接收端位置
            r_t = ArrayGeometry.get_rx_position(t);
            
            % 计算LoS路径的实际方向角
            [theta_los, phi_los] = ArrayGeometry.calculate_angles(r_t);
            
            % 添加平滑变化的角度偏移
            t_var = 0.1;  % 时间变化常数
            angle_var = pi/12 * (1 - exp(-t/t_var));  % 平滑变化的角度偏移
            
            % 设置初始偏移角（较小的值）
            init_offset = pi/180;  % 1度的初始偏移
            
            % 设置基础方向角，确保反射路径从接近LoS角度开始
            thetas = [theta_los,                  theta_los;                  % LoS路径
                     theta_los + init_offset,     theta_los - init_offset;   % 反射路径1初始位置
                     theta_los - init_offset,     theta_los + init_offset];  % 反射路径2初始位置
            
            % 添加时变偏移
            thetas(2:3,:) = thetas(2:3,:) + angle_var;  % 仅对反射路径添加时变偏移
            
            % 俯仰角设置类似方式处理
            phis = [phi_los,                    phi_los;                    % LoS路径
                   phi_los + init_offset/2,     phi_los - init_offset/2;   % 反射路径1初始位置
                   phi_los - init_offset/2,     phi_los + init_offset/2];  % 反射路径2初始位置
            
            % 添加时变偏移
            phis(2:3,:) = phis(2:3,:) + angle_var/2;  % 仅对反射路径添加时变偏移
        end
        
        function L = path_loss(r, is_sensing)
            cfg = Config;
            if nargin < 2
                is_sensing = false;
            end
            
            if is_sensing
                % 感知系统的路径损耗
                L = (cfg.c/(4*pi*cfg.f_sens*r)) * ...
                    exp(-cfg.k_f_sens*r) * ...
                    exp(-1j*2*pi*r/cfg.lambda_sens);
            else
                % 通信系统的路径损耗
                L = (cfg.c/(4*pi*cfg.f*r)) * ...
                    exp(-cfg.k_f*r) * ...
                    exp(-1j*2*pi*r/cfg.lambda);
            end
        end
        
        function a = steering_vector_2d(theta, phi, is_sensing)
            cfg = Config;
            if nargin < 3
                is_sensing = false;
            end
            
            a = zeros(cfg.Nx*cfg.Nz, 1);
            idx = 1;
            
            % 选择相应的波长和天线间距
            if is_sensing
                wavelength = cfg.lambda_sens;
                d = cfg.d_sens;
            else
                wavelength = cfg.lambda;
                d = cfg.d;
            end
            
            for nz = 0:(cfg.Nz-1)
                for nx = 0:(cfg.Nx-1)
                    % 计算局部坐标
                    x_m = (nx - (cfg.Nx-1)/2) * d;
                    z_n = (nz - (cfg.Nz-1)/2) * d;
                    
                    % 计算相位
                    phase = (2*pi/wavelength) * ...
                        (x_m * sin(theta)*cos(phi) + ...
                         z_n * sin(phi));
                    
                    a(idx) = exp(1j*phase);
                    idx = idx + 1;
                end
            end
            
            % 归一化
            a = a/sqrt(length(a));
        end
    end
end
