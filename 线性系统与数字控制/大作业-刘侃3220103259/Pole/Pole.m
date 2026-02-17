%% 极点配置控制系统设计
clear; close all; clc;

%% 1. 系统定义
% 连续时间状态空间矩阵
A = [0   0     0       1     0    0;
     0   0     0       0     1    0;
     0   0     0       0     0    1;
     0   2943/1600  327/1600  -7/72   0    0;
     0   8829/128  20601/640  -5/16   0    0;
     0   -61803/640  -10791/128  5/48   0    0];
B = [0;
     0;
     0;
     35/36;
     25/8;
     -25/24];
C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];
D = zeros(3,1);

% 显示连续时间系统特征值
cont_eigs = eig(A);

%% 2. 离散化
Ts = 0.1;  % 采样时间 (秒)
sysc = ss(A, B, C, D);  % 创建连续时间状态空间模型
sysd = c2d(sysc, Ts);  % 使用零阶保持法离散化
G = sysd.A;  % 离散状态矩阵
H = sysd.B;  % 离散输入矩阵
Cd = sysd.C; % 离散输出矩阵
Dd = sysd.D; % 离散直接传输矩阵

disc_eigs = eig(G);

%% 3. 控制器设计: 极点配置

% 设计参数
zeta = 0.71;      % 阻尼比
wn = [3, 5, 9];  % 自然频率 (rad/s) - 调整以确保稳定性和性能

% 连续时间期望极点 (三组共轭复数极点)
cont_poles = [];
for i = 1:length(wn)
    % 计算共轭极点对
    pole_pair = [
        -zeta*wn(i) + 1i*wn(i)*sqrt(1-zeta^2);
        -zeta*wn(i) - 1i*wn(i)*sqrt(1-zeta^2)
    ];
    cont_poles = [cont_poles; pole_pair];
end

% 映射到离散时间极点: s → z = e^(s*Ts)
disc_poles = exp(cont_poles * Ts);

disp('期望连续时间极点:');
disp(cont_poles);
disp('期望离散时间极点:');
disp(disc_poles);

% 使用极点配置计算状态反馈增益
K_pp = place(G, H, disc_poles);

disp('极点配置状态反馈增益:');
disp(K_pp);

%% 4. 构建闭环系统并计算脉冲响应
% 创建闭环系统状态空间模型 (以干扰d为输入)
A_cl = G - H*K_pp;  % 闭环状态矩阵
B_cl = H;           % 输入矩阵不变
% 扩展输出矩阵以包含控制输入 u = -Kx
C_cl_ext = Cd;  
D_cl_ext = Dd;     % 直接传输项扩展

% 构建闭环系统
sys_cl = ss(A_cl, B_cl, C_cl_ext, D_cl_ext, Ts);

% 计算脉冲响应
Tf = 10;                 % 仿真总时间 (秒)
t_sim = 0:Ts:Tf;         % 时间向量
[y_impulse, t_impulse] = impulse(sys_cl, t_sim);
y_impulse = squeeze(y_impulse); % 移除单维度

% 提取状态和控制输入
x1 = y_impulse(:,1);    % 小车位置x
x2 = y_impulse(:,2);    % 摆杆角度φ
x3 = y_impulse(:,3);    % 摆杆角度θ

%% 5. 结果绘图
% 图1: 三个位置状态分别绘图
figure('Name', '脉冲响应: 位置状态', 'Position', [100, 100, 1000, 800]);

% 小车位置x
subplot(3,1,1);
stairs(t_impulse, x1, 'r', 'LineWidth', 1.5); 
title('位置 (m)');
xlabel('时间 (秒)'); ylabel('位置 (m)');
grid on;

% 摆杆角度φ
subplot(3,1,2);
stairs(t_impulse, x2, 'g', 'LineWidth', 1.5);
title('摆杆角度 \phi');
xlabel('时间 (秒)'); ylabel('角度 (rad)');
grid on;

% 摆杆角度θ
subplot(3,1,3);
stairs(t_impulse, x3, 'b', 'LineWidth', 1.5);
title('摆杆角度 \theta');
xlabel('时间 (秒)'); ylabel('角度 (rad)');
grid on;


% 图2: 极点位置
figure('Name', '极点分布', 'Position', [300, 300, 1000, 500]);

% 开环极点
subplot(1,2,1);
plot(real(disc_eigs), imag(disc_eigs), 'go', 'MarkerSize', 10, 'LineWidth', 2);
hold on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--');  % 单位圆
axis equal;
grid on;
title('开环极点');
xlabel('实部'); ylabel('虚部');
xlim([-1.2 1.2]); ylim([-1.2 1.2]);

% 闭环极点
subplot(1,2,2);
cl_poles = eig(G - H*K_pp);  % 计算闭环极点
plot(real(cl_poles), imag(cl_poles), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hold on;
plot(cos(theta), sin(theta), 'k--');  % 单位圆
axis equal;
grid on;
title('闭环极点');
xlabel('实部'); ylabel('虚部');
xlim([-1.2 1.2]); ylim([-1.2 1.2]);

%% 6. 性能评估
% 峰值计算
peak_x1 = max(abs(x1));
peak_x2 = max(abs(x2));
peak_x3 = max(abs(x3));

fprintf('\n脉冲响应性能指标：\n');
fprintf('小车位置峰值：%.4f m\n', peak_x1);
fprintf('摆杆角度φ峰值：%.4f rad\n', peak_x2);
fprintf('摆杆角度θ峰值：%.4f rad\n', peak_x3);