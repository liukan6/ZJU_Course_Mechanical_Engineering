% 绘制瞬态性能区域 (Re vs Im)
close all; clear; clc;

% 计算约束参数
zeta_min = sqrt(1/((pi/log(5))^2 + 1));  % 对应 OS=20%
phi = acos(zeta_min);                    % rad
s_left = -8;                             % 实部界限

% 绘制阻尼线（从原点向外延伸）
r = linspace(0, 50, 200);
theta1 = pi - phi;       % 上半平面射线方向 (rad)
theta2 = - (pi - phi);   % 下半平面

% 两条阻尼线坐标
x1 = r .* cos(theta1); y1 = r .* sin(theta1);
x2 = r .* cos(theta2); y2 = r .* sin(theta2);

% 计算填充区域边界
% 从 Re=-50 到 Re=-8，沿上下边界计算 Im
x_fill = linspace(-50, s_left, 400);
y_upper = -x_fill * tan(phi);
y_lower = x_fill * tan(phi);

% 创建图形
figure; hold on; grid on; axis equal;

% 填充允许区域（蓝色淡填充）
fill([x_fill, fliplr(x_fill)], [y_upper, fliplr(y_lower)], ...
     [0.7 0.8 1], 'FaceAlpha', 0.6, 'EdgeColor', 'none');

% 绘制两条阻尼边界线（绿色虚线）
plot(x1, y1, 'g--', 'LineWidth', 1.5);
plot(x2, y2, 'g--', 'LineWidth', 1.5);

% 绘制垂直线 Re = -8（橙色虚线）
plot([s_left s_left], [-40 40], 'Color', [1 0.5 0], 'LineStyle', '--', 'LineWidth', 2);

% 计算并标记交点： -8 +/- j * 8 * tan(phi)
imag_bound = abs(s_left) * tan(phi);
plot(s_left, imag_bound, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);
plot(s_left, -imag_bound, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);

% 绘制坐标轴
plot([-50 20], [0 0], 'k-', 'LineWidth', 1);    % x轴
plot([0 0], [-40 40], 'k-', 'LineWidth', 1);   % y轴

% 添加标签与标题
xlabel('Re(s)', 'FontSize', 12);
ylabel('Im(s)', 'FontSize', 12);
title('瞬态性能允许区域: 在 Re <= -8 且 zeta >= zeta_min 内', 'FontSize', 13);
xlim([-50 20]);
ylim([-40 40]);

legend({'允许区域','\zeta = \zeta_{min} 边界','Re = -8 边界','交点','Location','bestoutside'});

hold off;
