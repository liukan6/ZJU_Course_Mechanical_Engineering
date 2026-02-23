%% =========== 多工况控制系统性能分析 ============
clear; close all; clc;

% 初始化数据存储
ControlResultData = [];
bandwidth_values = [5, 15, 25, 5, 15, 25];
initial_angles = [6, 6, 6, 10, 10, 10];

% 加载所有工况数据
for case_id = 1:6
    load("C\C_CtrlResData_" + case_id + ".mat");
    ControlResultData = [ControlResultData CtrlResData];
end

% 性能指标预分配
total_cases = numel(ControlResultData);
case_numbers = (1:total_cases).';
initial_theta_deg = NaN(total_cases, 1);
system_bandwidth = NaN(total_cases, 1);
settling_time_2percent = NaN(total_cases, 1);
overshoot_percentage = NaN(total_cases, 1);
control_signal_peak = NaN(total_cases, 1);
control_signal_rms = NaN(total_cases, 1);
response_rise_time = NaN(total_cases, 1);

% 各工况性能分析及图形绘制
figure_collection = gobjects(total_cases, 1);

for case_num = 1:total_cases
    current_case = ControlResultData(case_num);
    time_vector = current_case.time(:);
    
    % 信号提取
    reference_value = current_case.Yd(end);
    feedback_signal = current_case.FedY(:);
    control_signal = current_case.uout(:);
    alpha_angle = current_case.alpha(:);
    theta_angle = current_case.theta(:);

    % 性能指标计算
    initial_theta_deg(case_num) = theta_angle(1);
    system_bandwidth(case_num) = bandwidth_values(case_num);
    settling_time_2percent(case_num) = calculate_settling_time(time_vector, theta_angle, reference_value);
    overshoot_percentage(case_num) = calculate_overshoot(theta_angle, reference_value);
    control_signal_peak(case_num) = max(abs(control_signal));
    control_signal_rms(case_num) = sqrt(mean(control_signal.^2));
    response_rise_time(case_num) = calculate_rise_time(time_vector, theta_angle, reference_value);

    % 生成工况分析图
    fig_handle = figure('Name', sprintf('工况%d: 带宽=%g, 初值=%.1f°', ...
        case_num, bandwidth_values(case_num), initial_angles(case_num)), ...
        'NumberTitle', 'off', 'Color', 'white');
    figure_collection(case_num) = fig_handle;
    
    % 子图1: 控制输入信号
    subplot(3, 1, 1);
    plot(time_vector, control_signal, 'LineWidth', 1.5); grid on;
    ylabel('控制量 u'); xlabel('时间 (s)');
    title(sprintf('工况 %d - 控制输入信号', case_num));
    control_info = sprintf('峰值=%.3g, 均方根=%.3g', ...
        control_signal_peak(case_num), control_signal_rms(case_num));
    xlim([time_vector(1), time_vector(end)]);
    legend(control_info, 'Location', 'best');

    % 子图2: 旋转臂角度
    subplot(3, 1, 2);
    plot(time_vector, alpha_angle, 'LineWidth', 1.5); grid on;
    ylabel('\alpha (度)'); xlabel('时间 (s)');
    title('旋转臂角度 \alpha(t)');

    % 子图3: 摆杆角度及参考值
    subplot(3, 1, 3);
    plot(time_vector, theta_angle, 'b', 'LineWidth', 1.5); hold on;
    plot(time_vector, reference_value * ones(size(time_vector)), 'r--', 'LineWidth', 1.2);
    grid on; xlabel('时间 (s)'); ylabel('\theta (度)');
    performance_info = sprintf('\\theta(t)');
    title(performance_info);

    % 图形布局调整
    set(fig_handle, 'Position', [200, 50, 700, 800]);
    drawnow;
end

% 摆杆角度对比图
theta_comparison_fig = figure('Name', '所有工况摆杆角度对比', ...
    'NumberTitle', 'off', 'Color', 'white');
color_palette = lines(total_cases);
hold on; 
grid on;

for case_num = 1:total_cases
    plot(ControlResultData(case_num).time, ControlResultData(case_num).theta, ...
        'Color', color_palette(case_num, :), 'LineWidth', 1.5);
end

legend_labels = arrayfun(@(idx) sprintf('工况%d: 带宽=%g, 初值=%g°', ...
    idx, bandwidth_values(idx), initial_angles(idx)), 1:total_cases, 'UniformOutput', false);
legend(legend_labels, 'Location', 'best');
xlabel('时间 (s)'); ylabel('\theta (度)');
title('各工况摆杆角度 \theta(t) 对比');
set(theta_comparison_fig, 'Position', [100, 100, 900, 450]);

% 旋转臂角度对比图
alpha_comparison_fig = figure('Name', '所有工况旋转臂角度对比', ...
    'NumberTitle', 'off', 'Color', 'white');
hold on; grid on;

for case_num = 1:total_cases
    time_data = ControlResultData(case_num).time;
    alpha_data = ControlResultData(case_num).alpha;
    plot(time_data, alpha_data, 'Color', color_palette(case_num, :), 'LineWidth', 1.5);
end

legend(legend_labels, 'Location', 'best');
xlabel('时间 (s)'); ylabel('\alpha (度)');
title('各工况旋转臂角度 \alpha(t) 对比');
set(alpha_comparison_fig, 'Position', [100, 100, 900, 450]);

% 性能汇总表
PerformanceSummary = table(case_numbers, system_bandwidth, initial_theta_deg, ...
    overshoot_percentage, response_rise_time, settling_time_2percent, ...
    control_signal_peak, control_signal_rms, ...
    'VariableNames', {'工况编号', '系统带宽', '初始角度_度', '超调量_百分比', ...
    '上升时间_秒', '调节时间_秒', '控制量峰值', '控制量均方根'});

disp('六种工况性能汇总表:');
disp(PerformanceSummary);

% 图形保存选项
enable_figure_saving = true;
if enable_figure_saving
    for case_num = 1:total_cases
        saveas(figure_collection(case_num), ...
            sprintf('Case%d_PerformanceAnalysis.png', case_num));
    end
    saveas(theta_comparison_fig, 'AllCases_ThetaComparison.png');
    saveas(alpha_comparison_fig, 'AllCases_AlphaComparison.png');
    fprintf('所有图形已保存.\n');
end

%% =========== 局部函数定义 ============

function settling_time = calculate_settling_time(time_vec, output_signal, reference_val)
    % 计算2%调节时间
    if abs(reference_val) < 1e-9
        tolerance = 0.02 * max(abs(output_signal));
        if tolerance < 1e-6
            tolerance = 1e-6;
        end
    else
        tolerance = 0.02 * abs(reference_val);
    end
    
    error_signal = abs(output_signal - reference_val);
    exceeding_indices = find(error_signal > tolerance);
    
    if isempty(exceeding_indices)
        settling_time = 0;
    else
        last_exceed_index = exceeding_indices(end);
        if last_exceed_index >= length(time_vec)
            settling_time = NaN;
        else
            settling_time = time_vec(last_exceed_index + 1);
        end
    end
end

function overshoot = calculate_overshoot(output_signal, reference_val)
    % 计算超调量百分比
    if abs(reference_val) < 1e-9
        overshoot = NaN;
        return;
    end
    
    maximum_value = max(output_signal);
    overshoot = (maximum_value - reference_val) / abs(reference_val) * 100;
    
    if overshoot < 0
        overshoot = 0;
    end
end

function rise_time = calculate_rise_time(time_vec, output_signal, reference_val)
    % 计算10%-90%上升时间
    if abs(reference_val) < 1e-9
        rise_time = NaN;
        return;
    end
    
    initial_value = output_signal(1);
    ten_percent_level = initial_value + 0.1 * (reference_val - initial_value);
    ninety_percent_level = initial_value + 0.9 * (reference_val - initial_value);
    
    idx_10 = find((output_signal - initial_value) .* (reference_val - initial_value) >= 0 & ...
        abs(output_signal - initial_value) >= abs(ten_percent_level - initial_value), 1);
    idx_90 = find((output_signal - initial_value) .* (reference_val - initial_value) >= 0 & ...
        abs(output_signal - initial_value) >= abs(ninety_percent_level - initial_value), 1);
    
    if isempty(idx_10) || isempty(idx_90)
        rise_time = NaN;
    else
        rise_time = time_vec(idx_90) - time_vec(idx_10);
    end
end