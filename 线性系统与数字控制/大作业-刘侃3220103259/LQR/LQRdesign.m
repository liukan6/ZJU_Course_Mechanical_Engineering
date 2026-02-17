%% 离散LQR控制器设计
clear; clc;
load("state_space.mat");
load("LQRpara.mat");
[K_d,S_d,P_d]=dlqr(sysd.A,sysd.B,Q,R);

%% 绘制离散系统脉冲响应（阶梯图）
figure;
set(gcf, 'Position', [100, 100, 800, 600], 'Color', 'w');

% 创建闭环系统
sys_cl = ss(sysd.A - sysd.B * K_d, sysd.B, sysd.C, sysd.D, Ts);

% 计算脉冲响应
Tf=10;
t_sim = 0:Ts:Tf;
[y, t] = impulse(sys_cl, t_sim);

% 绘制三个输出的阶梯图
subplot(3,1,1);
stairs(t, y(:,1), 'r', 'LineWidth', 1.5);
hold on;
grid on;
title('小车位置响应', 'FontSize', 12);
ylabel('位置 (m)', 'FontSize', 10);
legend('阶梯响应', 'Location', 'best');
xlim([0, max(t)]);

subplot(3,1,2);
stairs(t, y(:,2), 'g', 'LineWidth', 1.5);
hold on;
grid on;
title('摆杆角度 \phi', 'FontSize', 12);
ylabel('角度 (°)', 'FontSize', 10);
legend('阶梯响应', 'Location', 'best');
xlim([0, max(t)]);

subplot(3,1,3);
stairs(t, y(:,3), 'b', 'LineWidth', 1.5);
hold on;
grid on;
title('摆杆角度 \theta', 'FontSize', 12);
xlabel('时间 (秒)', 'FontSize', 10);
ylabel('角度 (°)', 'FontSize', 10);
legend('阶梯响应', 'Location', 'best');
xlim([0, max(t)]);

%% 性能评估
% 峰值计算
peak_x1 = max(abs(y(:,1)));
peak_x2 = max(abs(y(:,2)));
peak_x3 = max(abs(y(:,3)));

fprintf('\n脉冲响应性能指标：\n');
fprintf('小车位置峰值：%.4f m\n', peak_x1);
fprintf('摆杆角度φ峰值：%.4f rad\n', peak_x2);
fprintf('摆杆角度θ峰值：%.4f rad\n', peak_x3);