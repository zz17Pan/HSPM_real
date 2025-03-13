% plotResults.m - 结果可视化函数

function plotResults(t_vec, capacity_history, angles_history, angles_history_los, ...
                    angles_history_ref1, angles_history_ref2, distance_history, cfg)
    figure('Position', [100 100 1200 800]);
    
    % 1. 信道容量随时间变化
    subplot(2,2,1);
    plot(t_vec, capacity_history, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('时间 (s)');
    ylabel('信道容量 (bits/s/Hz)');
    title('信道容量随时间变化');
    
    % 2. 方向角随时间变化
    subplot(2,2,2);
    % 将角度转换为度数并确保在[0, 360]范围内
    plot(t_vec, mod(angles_history_los(:,1)*180/pi, 360), 'b-', 'LineWidth', 2); hold on;
    %plot(t_vec, mod(angles_history_ref1(:,1)*180/pi, 360), 'r--', 'LineWidth', 2);
    %plot(t_vec, mod(angles_history_ref2(:,1)*180/pi, 360), 'g:', 'LineWidth', 2);
    grid on;
    xlabel('时间 (s)');
    ylabel('方位角 (度)');
    title('多路径方位角随时间变化');
    legend('LoS路径', '反射路径1', '反射路径2', 'Location', 'best');
    ylim([0, 360]);
    
    % 3. 通信距离随时间变化
    subplot(2,2,3);
    plot(t_vec, distance_history, 'k-', 'LineWidth', 2);
    grid on;
    xlabel('时间 (s)');
    ylabel('距离 (m)');
    title('通信距离随时间变化');
    
    % 4. 接收端轨迹（xy平面投影）
    subplot(2,2,4);
    rx_pos = zeros(length(t_vec), 3);
    for i = 1:length(t_vec)
        rx_pos(i,:) = ArrayGeometry.get_rx_position(t_vec(i));
    end
    plot(rx_pos(:,1), rx_pos(:,2), 'r-', 'LineWidth', 2);
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');
    title('接收端运动轨迹 (xy平面投影)');
    axis equal;
    
    % 添加总标题
    sgtitle(sprintf('太赫兹MIMO-HSPM系统性能分析\nD0 = %d m, SNR = %d dB', ...
            cfg.D0, cfg.SNR_dB));
end