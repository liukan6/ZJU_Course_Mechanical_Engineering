clc;
clear;
close all;

% 采样时间
T = 0.1;

% 定义 s 平面的扇形区域 (阻尼比和自然频率)
zeta = linspace(0.5, 0.9, 100);      % ζ ∈ [0.5, 0.9]
omega_n = linspace(0, 20, 100);       % ω_n ∈ [0, 20]
[zeta_grid, omega_n_grid] = meshgrid(zeta, omega_n);

% 计算 s 的实部和虚部
sigma = -zeta_grid .* omega_n_grid;
omega = omega_n_grid .* sqrt(1 - zeta_grid.^2);

% 生成 s 和 z 的上下共轭点
s1 = sigma + 1i * omega;
s2 = sigma - 1i * omega;
z1 = exp(T * s1);
z2 = exp(T * s2);

% 绘制 s 平面
figure;
subplot(1, 2, 1);
plot(real(s1(:)), imag(s1(:)), '.b');
hold on;
plot(real(s2(:)), imag(s2(:)), '.b');
xlabel('Real(s) = \sigma');
ylabel('Imag(s) = j\omega');
title('s-plane');
grid on;
axis equal;
xlim([-25, 5]);
ylim([-25, 25]);

% 绘制 z 平面
subplot(1, 2, 2);
plot(real(z1(:)), imag(z1(:)), '.r');
hold on;
plot(real(z2(:)), imag(z2(:)), '.r');
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