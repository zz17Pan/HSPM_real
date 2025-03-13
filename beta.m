
    %% 1. 系统参数提取（从Config类）
    f = 3e11;            % 300 GHz
    c = 3e8;             % 光速
    lambda = 0.001;      % 波长 1mm
    k_f = 0.1;           % 吸收系数
    D0 = 40;             % 参考距离
    SNR_dB = 10;         % 目标SNR
    
    %% 2. 发射端参数
    % 发射功率（太赫兹系统典型值）
    Pt_dBm = 10;         % 10 dBm = 10 mW (太赫兹发射功率通常较小)
    Pt = 10^((Pt_dBm-30)/10);  % 转换为瓦特
    
    %% 3. 天线增益计算
    % 单个子阵天线数
    Nx = 4; Nz = 4;      % 4×4子阵
    N_element = Nx * Nz; % 每个子阵16个元素
    
    % 子阵数量
    Kt = 4; Kr = 4;      % 发射接收各4个子阵
    
    % 理论最大方向性增益(dBi)
    Gt_dB = 10*log10(N_element * Kt);  % 发射增益
    Gr_dB = 10*log10(N_element * Kr);  % 接收增益
    
    % 转换为线性值
    G = 10^((Gt_dB + Gr_dB)/20);
    
    %% 4. 路径损耗计算
    % 自由空间损耗
    FSL_dB = 20*log10(4*pi*D0/lambda);
    
    % 分子吸收损耗
    ML_dB = k_f * D0 * 10*log10(exp(1));
    
    % 总路径损耗
    PL_dB = FSL_dB + ML_dB;
    PL = 10^(PL_dB/10);
    
    %% 5. 噪声功率计算
    k_B = 1.38e-23;      % 玻尔兹曼常数
    T0 = 290;            % 噪声温度 (K)
    B = 2e9;             % 带宽 (2 GHz)
    N0 = k_B * T0 * B;   % 噪声功率
    
    %% 6. Beta计算
    % 考虑所需SNR
    SNR_lin = 10^(SNR_dB/10);
    required_Pr = SNR_lin * N0;
    
    % Beta计算（考虑路径损耗和天线增益）
    Beta = sqrt(required_Pr * PL / (Pt * G));
    
    %% 7. 验证计算结果
    % 计算实际接收功率
    Pr = Pt * G * Beta^2 / PL;
    Pr_dBm = 10*log10(Pr*1000);
    actual_SNR = 10*log10(Pr/N0);
    
    % 输出结果
    fprintf('===== Beta计算结果 =====\n');
    fprintf('发射功率: %.2f dBm\n', Pt_dBm);
    fprintf('天线总增益: %.2f dBi\n', Gt_dB + Gr_dB);
    fprintf('路径损耗: %.2f dB\n', PL_dB);
    fprintf('Beta值: %.2e\n', Beta);
    fprintf('接收功率: %.2f dBm\n', Pr_dBm);
    fprintf('实际SNR: %.2f dB\n', actual_SNR);
    
