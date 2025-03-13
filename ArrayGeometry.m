classdef ArrayGeometry
    methods (Static)
        function [tx_pos, rx_pos] = initialize_array(t)
            try
                cfg = Config;
                
                % 初始化发射端位置矩阵（固定在原点）
                tx_pos = zeros(cfg.Nt_total, 3);
                idx = 1;
                for kt = 1:cfg.Kt
                    % 计算子阵中心位置
                    center_x = (kt - (cfg.Kt+1)/2) * cfg.d_sub;
                    
                    for nz = 0:(cfg.Nz-1)
                        for nx = 0:(cfg.Nx-1)
                            % 计算局部坐标
                            local_x = (nx - (cfg.Nx-1)/2) * cfg.d;
                            local_z = (nz - (cfg.Nz-1)/2) * cfg.d;
                            
                            % 全局坐标
                            tx_pos(idx,:) = [center_x + local_x, 0, local_z];
                            idx = idx + 1;
                        end
                    end
                end
                
                % 获取接收端当前位置
                r_t = ArrayGeometry.get_rx_position(t);
                
                % 初始化接收端位置矩阵
                rx_pos = zeros(cfg.Nr_total, 3);
                idx = 1;
                for kr = 1:cfg.Kr
                    % 计算子阵中心位置
                    center_x = r_t(1) + (kr - (cfg.Kr+1)/2) * cfg.d_sub;
                    
                    for nz = 0:(cfg.Nz-1)
                        for nx = 0:(cfg.Nx-1)
                            % 计算局部坐标
                            local_x = (nx - (cfg.Nx-1)/2) * cfg.d;
                            local_z = (nz - (cfg.Nz-1)/2) * cfg.d;
                            
                            % 全局坐标
                            rx_pos(idx,:) = [center_x + local_x, r_t(2), r_t(3) + local_z];
                            idx = idx + 1;
                        end
                    end
                end
                
                % 输出调试信息
                fprintf('阵列初始化完成 (t = %.3f s):\n', t);
                fprintf('发射阵列维度: [%d, %d]\n', size(tx_pos));
                fprintf('接收阵列维度: [%d, %d]\n', size(rx_pos));
                
            catch ME
                fprintf('阵列初始化失败：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:40:33', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function r_t = get_rx_position(t)
            try
                % 计算接收端在时刻t的位置
                cfg = Config;
                
                % 定义初始位置 - 修改初始x坐标为小正数，避免初始方位角为180度
                r0 = [0.1, 0, cfg.D0];  % 将初始x坐标设为0.1m
                
                % 定义速度向量 - 降低初始速度，实现平滑加速
                t_acc = 0.2;  % 加速时间常数
                vx_max = 30;  % 最大x方向速度
                vy_max = 20;  % 最大y方向速度
                
                % 使用平滑加速函数
                vx = vx_max * (1 - exp(-t/t_acc));
                vy = vy_max * (1 - exp(-t/t_acc));
                vz = 0;
                
                % 计算位置 - 通过积分得到平滑轨迹
                x_t = r0(1) + vx_max * (t + t_acc * (exp(-t/t_acc) - 1));
                y_t = r0(2) + vy_max * (t + t_acc * (exp(-t/t_acc) - 1));
                z_t = r0(3);
                
                r_t = [x_t, y_t, z_t];
                
                % 确保不超过最大速度约束
                if norm([vx, vy, vz]) > cfg.v_max
                    v_norm = [vx, vy, vz] / norm([vx, vy, vz]) * cfg.v_max;
                    vx = v_norm(1);
                    vy = v_norm(2);
                    vz = v_norm(3);
                end
                
                % 输出调试信息
                fprintf('接收端位置 (t = %.3f s): [%.2f, %.2f, %.2f] m\n', t, r_t(1), r_t(2), r_t(3));
                fprintf('当前速度: [%.2f, %.2f, %.2f] m/s\n', vx, vy, vz);
                
            catch ME
                fprintf('接收端位置计算失败：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:40:33', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                rethrow(ME);
            end
        end
        
        function [theta, phi] = calculate_angles(vector)
            try
                % 计算方位角和俯仰角
                x = vector(1); y = vector(2); z = vector(3);
                r_xy = sqrt(x^2 + y^2);
                
                % 修改方位角计算，确保角度连续性
                theta = atan2(y, x);
                if theta < 0
                    theta = theta + 2*pi;  % 将角度范围调整到[0, 2π]
                end
                
                phi = atan2(z, r_xy);
                
                % 输出调试信息
                fprintf('角度计算结果:\n');
                fprintf('方位角: %.2f° (%.4f rad)\n', theta*180/pi, theta);
                fprintf('俯仰角: %.2f° (%.4f rad)\n', phi*180/pi, phi);
                
            catch ME
                fprintf('角度计算失败：\n');
                fprintf('错误信息：%s\n', ME.message);
                fprintf('时间：%s\n', datestr(datetime('2025-03-13 05:40:33', 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));
                fprintf('用户：zz17Pan\n');
                fprintf('输入向量: [%.2f, %.2f, %.2f]\n', vector(1), vector(2), vector(3));
                rethrow(ME);
            end
        end
    end
end