% Config.m - 系统配置

classdef Config
    properties (Constant)
        % 物理参数 - 通信系统 (300GHz)
        c = 3e8;                  % 光速 (m/s)
        lambda = 0.001;           % 通信波长 (1 mm)
        f = Config.c/Config.lambda; % 通信载波频率 (300 GHz)
        k_f = 0.1;               % 太赫兹衰收系数
        
        % 物理参数 - 感知系统 (10GHz)
        lambda_sens = 0.03;       % 感知波长 (30 mm)
        f_sens = Config.c/Config.lambda_sens; % 感知载波频率 (10 GHz)
        k_f_sens = 0.01;         % X波段衰减系数
        
        % 其他物理参数
        k_B = 1.38e-23;          % 玻尔兹曼常数
        T0 = 300;                % 噪声温度 (K)
        B = 2e9;                 % 带宽 (2 GHz)
        
        % 阵列配置
        Kt = 4;                   % 发射端子阵数量
        Kr = 4;                   % 接收端子阵数量
        Nx = 4;                   % 子阵x方向天线数
        Nz = 4;                   % 子阵z方向天线数
        N_at = Config.Nx * Config.Nz;  % 每个子阵天线总数
        N_ar = Config.Nx * Config.Nz;  % 每个子阵天线总数
        Nt_total = Config.Kt * Config.N_at;  % 发射端总天线数
        Nr_total = Config.Kr * Config.N_ar;  % 接收端总天线数
        
        % 几何参数
        d = Config.lambda/2;      % 通信天线间距
        d_sens = Config.lambda_sens/2; % 感知天线间距
        d_sub = 16*Config.lambda; % 子阵间距
        D0 = 40;                  % 初始距离 (m)
        
        % 通信参数
        SNR_dB = 10;             % SNR (dB)
        Beta = 2e9;              % 功率缩放因子
        
        % 运动参数
        v_max = 55.56;           % 最大速度 (200 km/h)
        Ts = 0.001;              % 采样周期 (1 ms)
        
        % 资源分配参数
        Kt_comm = 3;             % 通信用发射子阵数
        Kr_comm = 3;             % 通信用接收子阵数
        num_beams = 64;          % 波束码本大小
        theta_range = [-60, 60];  % 方位角范围(度)
        phi_range = [-30, 30];    % 俯仰角范围(度)
        
        % OFDM参数
        K_sc = 1024;             % 子载波数
        L_CP = 64;               % 循环前缀长度
        
        % 信道参数
        Np = 3;                   % 路径数（1个LoS + 2个反射）
    end
end