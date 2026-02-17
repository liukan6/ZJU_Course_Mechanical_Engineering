% 闭环离散传递函数
num = [0.4];                   % Numerator: 0.4z -> 0.4*z^1 + 0
den = [1 -1.2 0.7];         % Denominator: z^3 - 2.2z^2 + 1.9z - 0.7

Ts = 1;                          % 采样时间未知，假设为1（单位样本）
sys = tf(num, den, Ts);         % 创建离散系统

% 计算单位阶跃响应
N = 50;                          % 观察前50个采样点
[y, t] = step(sys, N);

% 找到第一个峰值的时间
[~, peakIndex] = max(y);        % 找到最大值的索引（首次峰值）
firstPeakTime = t(peakIndex);   % 取对应的时间（样本编号）

% 显示结果
fprintf('First peak occurs at k = %d\n', firstPeakTime);

% 绘图（使用实心点 + 连线）
figure;
plot(t, y, '-o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', ...
    'Color', 'b', 'LineWidth', 1.5); grid on;
title('Step Response of the Discrete-Time System');
xlabel('k (time steps)');
ylabel('Output y[k]');
hold on;

% 标记峰值
plot(firstPeakTime, y(peakIndex), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(firstPeakTime + 0.5, y(peakIndex), ...
    sprintf('Peak at k = %d', firstPeakTime), ...
    'Color', 'r', 'FontSize', 10);
