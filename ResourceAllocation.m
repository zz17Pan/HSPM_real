classdef ResourceAllocation
    methods (Static)
        function S_pilot = generate_pilot_signals(N_ant)
            % 生成正交导频信号
            if nargin < 1
                cfg = Config;
                N_ant = cfg.Nx * cfg.Nz;
            end
            
            % 生成导频信号
            S_pilot = (randn(N_ant, N_ant) + 1j*randn(N_ant, N_ant))/sqrt(2);
            
            % 正交化处理
            [Q,~] = qr(S_pilot);
            S_pilot = Q;
            
            % 验证维度
            fprintf('生成的导频信号维度: [%d, %d]\n', size(S_pilot));
        end
        
        function [comm_subarrays, sens_subarrays] = allocate_subarrays()
            cfg = Config;
            
            % 确保子阵索引不超过总子阵数
            max_kt = min(cfg.Kt_comm, cfg.Kt);
            max_kr = min(cfg.Kr_comm, cfg.Kr);
            
            % 通信子阵索引
            comm_subarrays.tx = 1:max_kt;
            comm_subarrays.rx = 1:max_kr;
            
            % 感知子阵索引 (最后一个子阵)
            sens_subarrays.tx = cfg.Kt;
            sens_subarrays.rx = cfg.Kr;
        end
        
        function mask = get_sensing_antenna_mask()
            cfg = Config;
            mask = zeros(cfg.Nx, cfg.Nz);
            
            % 保留中心行和列
            mid_x = ceil(cfg.Nx/2);
            mid_z = ceil(cfg.Nz/2);
            mask(mid_x,:) = 1;
            mask(:,mid_z) = 1;
        end
    end
end