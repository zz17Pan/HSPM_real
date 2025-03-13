% test_multimodal_tracking.m
clear all;
close all;
clc;

try
    % 记录开始时间
    start_time = datetime('2025-03-13 05:20:19', 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
    fprintf('测试开始时间: %s\n', datestr(start_time));
    fprintf('用户: zz17Pan\n\n');
    
    % 创建跟踪系统实例
    fprintf('初始化系统...\n');
    tracker = MultiModalTracking();
    
    % 设置仿真时长
    duration = 10;  % 10秒
    fprintf('设置仿真时长: %d 秒\n', duration);
    
    % 运行跟踪
    fprintf('开始仿真...\n');
    tracker.run_tracking(duration);
    
    % 显示结果
    fprintf('仿真完成，显示结果...\n');
    tracker.display_final_results();
    
    % 记录结束时间
    end_time = datetime('now', 'TimeZone', 'UTC');
    fprintf('\n测试结束时间: %s\n', datestr(end_time));
    fprintf('总运行时间: %s\n', char(end_time - start_time));
    
catch ME
    fprintf('错误：%s\n', ME.message);
    fprintf('位置：%s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
    fprintf('发生时间：%s\n', datestr(datetime('now', 'TimeZone', 'UTC')));
    fprintf('用户：zz17Pan\n');
end