clc;
clear;
close all;

% 采样时间
T = 0.1;

% 定义 s 平面的矩形区域
sigma = linspace(-20, -5, 100);   % σ ∈ [-20, -5]
omega = linspace(-6, 6, 100);     % ω ∈ [-6, 6]
[sigma_grid, omega_grid] = meshgrid(sigma, omega);
s = sigma_grid + 1i * omega_grid;

% 映射到 z 平面
z = exp(T * s);

% 绘制 s 平面
figure;
subplot(1, 2, 1);
plot(real(s(:)), imag(s(:)), '.b');
xlabel('Real(s) = \sigma');
ylabel('Imag(s) = j\omega');
title('s-plane');
grid on;
axis equal;
xlim([-25, 5]);
ylim([-10, 10]);

% 绘制 z 平面
subplot(1, 2, 2);
plot(real(z(:)), imag(z(:)), '.r');
hold on;
% 绘制单位圆
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--');
xlabel('Re(z)');
ylabel('Im(z)');
title('z-plane');
grid on;
axis equal;
xlim([-1.2, 1.2]);
ylim([-1.2, 1.2]);