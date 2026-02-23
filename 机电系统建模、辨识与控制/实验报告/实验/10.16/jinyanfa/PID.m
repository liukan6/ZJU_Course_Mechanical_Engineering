%% batch_pid_sim_simple.m
clc; clear; close all;

% 基本配置
Ts = 0.005;          % 采样时间（s）
step_final = 0.5;    % 阶跃终值
tmax = 10;           % 仿真总时长（s）
load('systempara.mat');

% 待测试 PID 参数组（每行 [Kp, Ki, Kd, tauD]） - 6 组
pid_sets = [
    20,  40,  1.5, 0.01;
    40,  80,  3.0, 0.01;
    60, 110,  8.0, 0.01;
    120, 80,  5.0, 0.01;
    80, 150, 4.0, 0.01;
    30, 200, 0.5, 0.01;
];

nCases = size(pid_sets,1);

% 结果表（简化）
Results = table('Size',[nCases 9],...
    'VariableTypes',{'string','double','double','double','double','double','double','double','double'},...
    'VariableNames',{'Name','Kp','Ki','Kd','RiseTime','Overshoot','SettlingTime','SteadyError','IAE'});

% 创建一个 3x2 的子图布局，每个子图画一条曲线
fig = figure('Name','PID Responses (each in own subplot)','NumberTitle','off');
tl = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

for i = 1:nCases
    Kp = pid_sets(i,1);
    Ki = pid_sets(i,2);
    Kd = pid_sets(i,3);
    tauD = pid_sets(i,4);

    % 运行仿真（只用 out = sim(...)）
    out = sim('simu.slx');

    % 假定 out.angle 是仅包含角度的向量（无时间信息）
    y = out.angle;
    y = y(:);
    t = (0:length(y)-1)' * Ts;
    idx = t <= tmax;
    t = t(idx); y = y(idx);

    % 指标计算（10%->90% 上升时间，超调，2% 稳定时间，稳态误差，IAE）
    target = step_final;

    t10i = find(y >= 0.1*target, 1, 'first');
    t90i = find(y >= 0.9*target, 1, 'first');
    if isempty(t10i) || isempty(t90i)
        rise_time = NaN;
    else
        rise_time = t(t90i) - t(t10i);
    end

    peak = max(y);
    overshoot = max(0, (peak - target)/target * 100);

    tol = 0.02 * target;
    outside = find(abs(y - target) > tol);
    if isempty(outside)
        settling_time = 0;
    else
        last_out = outside(end);
        if last_out == length(t)
            settling_time = NaN;
        else
            settling_time = t(last_out + 1);
        end
    end

    tail_start = max(1, floor(length(y)*0.95)+1);
    steady_err = mean(target - y(tail_start:end));

    IAE = trapz(t, abs(target - y));

    % 存表
    Results.Name(i) = sprintf('Case%d', i);
    Results.Kp(i) = Kp;
    Results.Ki(i) = Ki;
    Results.Kd(i) = Kd;
    Results.RiseTime(i) = rise_time;
    Results.Overshoot(i) = overshoot;
    Results.SettlingTime(i) = settling_time;
    Results.SteadyError(i) = steady_err;
    Results.IAE(i) = IAE;

    % 绘到对应子图
    ax = nexttile;
    plot(t, y, 'LineWidth', 1.2);
    hold on;
    yline(target,'--','Target','LabelHorizontalAlignment','left');
    xlabel('Time (s)');
    ylabel('Angle');
    title(sprintf('Case %d: Kp=%.0f Ki=%.0f Kd=%.1f', i, Kp, Ki, Kd),'Interpreter','none');

    % 在子图上显示性能指标文本（右上角）
    metrics_str = {
        sprintf('Rise: %s s', num2str(rise_time,'%.3g'));
        sprintf('OS: %.2f %%', overshoot);
        sprintf('Settle: %s s', num2str(settling_time,'%.3g'));
        sprintf('SteadyErr: %.3g', steady_err);
        sprintf('IAE: %.3g', IAE)
        };
    xlim_vals = xlim;
    ylim_vals = ylim;
    tx = xlim_vals(1) + 0.02*(xlim_vals(2)-xlim_vals(1));
    ty = ylim_vals(2) - 0.06*(ylim_vals(2)-ylim_vals(1));
    text(tx, ty, metrics_str, 'HorizontalAlignment','left', 'VerticalAlignment','top', 'FontSize',9, 'BackgroundColor','none');

    grid on;
    hold off;
end

% 输出与保存
disp('Results summary:');
disp(Results);

