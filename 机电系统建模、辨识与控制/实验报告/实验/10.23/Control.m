%% 状态方程
clear;clc;close all;
load("模型参数.mat");
p = I*(M + m) + M*m*l^2;
A = [0, 1, 0, 0;
     (m^2*g*l^2)/p, 0, 0, (m*l*b)/p;
     0, 0, 0, 1;
     -(m*g*l*(M + m))/p, 0, 0, -(I + m*l^2)*b/p];

B = [0; -(m*l)/p; 0; (I + m*l^2)/p];
C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
D = [0; 0; 0; 0];

%% 1. 检验系统的可控性
fprintf('=== 1. 系统可控性检验 ===\n');
Co = ctrb(A, B);
rank_Co = rank(Co);
fprintf('可控性矩阵的秩: %d\n', rank_Co);
fprintf('系统状态维度: %d\n', length(A));

if rank_Co == length(A)
    fprintf('系统完全可控，可以进行极点配置\n');
else
    fprintf('系统不完全可控，需要检查模型\n');
    return;
end

%% 2. 检查开环系统特性
fprintf('\n=== 2. 开环系统分析 ===\n');
open_loop_poles = eig(A);
fprintf('开环系统极点:\n');
disp(open_loop_poles);

% 检查是否有右半平面极点（不稳定的标志）
if any(real(open_loop_poles) > 0)
    fprintf('开环系统不稳定！这是倒立摆的典型特性。\n');
end

%% 3. 重新设计极点配置
fprintf('\n=== 3. 极点配置控制器重新设计 ===\n');

% 对于倒立摆系统，我们需要更保守的极点配置
% 主导极点应该具有足够的稳定裕度

% 方法1: 基于性能指标的极点配置
ts = 2;        % 调整时间 2秒
Mp = 0.10;     % 最大超调量 10%

% 计算二阶系统的阻尼比和自然频率
zeta = -log(Mp) / sqrt(pi^2 + log(Mp)^2);
wn = 4 / (zeta * ts);

fprintf('期望性能指标:\n');
fprintf('调整时间 ts = %.2f 秒\n', ts);
fprintf('最大超调量 Mp = %.1f%%\n', Mp*100);
fprintf('计算得到的阻尼比 zeta = %.3f\n', zeta);
fprintf('计算得到的自然频率 wn = %.3f rad/s\n', wn);

% 主导极点（复数对）- 确保在左半平面
dominant_poles = [-zeta*wn + 1i*wn*sqrt(1-zeta^2), -zeta*wn - 1i*wn*sqrt(1-zeta^2)];

% 另外两个极点配置为更远离虚轴，但不要太快以免控制力过大
other_poles = [-10 - 0.0001*1i, -10 + 0.0001*1i];  % 比之前更保守

% 期望极点集合
desired_poles = [dominant_poles, other_poles];
fprintf('\n新的期望极点配置:\n');
disp(desired_poles);

% 使用极点配置方法计算反馈增益矩阵 K
K_new = place(A, B, desired_poles);
fprintf('新的反馈增益矩阵 K_new:\n');
disp(K_new);

%% 4. 使用初始条件响应测试（更适合倒立摆）
fprintf('\n=== 4. 初始条件响应测试 ===\n');

% 创建闭环系统
A_cl_new = A - B*K_new;
sys_cl_new = ss(A_cl_new, B, C, D);

% 测试初始角度扰动（更符合实际情况）
x0 = [0.1; 0; 0; 0];  % 初始角度0.1弧度，其他状态为0

% 模拟初始条件响应
t = 0:0.01:10;  % 10秒仿真时间
[y, t, x] = initial(sys_cl_new, x0, t);

% 绘制响应曲线
figure('Position', [100, 100, 1200, 800]);

subplot(2,2,1);
plot(t, x(:,1), 'b', 'LineWidth', 2);
title('摆杆角度 θ 响应', 'Interpreter', 'none');
xlabel('时间 (s)'); ylabel('角度 (rad)');
grid on;

subplot(2,2,2);
plot(t, x(:,2), 'b', 'LineWidth', 2);
title('角速度 dθ/dt 响应', 'Interpreter', 'none');
xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
grid on;

subplot(2,2,3);
plot(t, x(:,3), 'b', 'LineWidth', 2);
title('小车位移 x 响应', 'Interpreter', 'none');
xlabel('时间 (s)'); ylabel('位移 (m)');
grid on;

subplot(2,2,4);
plot(t, x(:,4), 'b', 'LineWidth', 2);
title('小车速度 dx/dt 响应', 'Interpreter', 'none');
xlabel('时间 (s)'); ylabel('速度 (m/s)');
grid on;

sgtitle('倒立摆系统初始条件响应 (θ₀ = 0.1 rad)', 'Interpreter', 'none');

%% 5. 分析控制性能
fprintf('\n=== 5. 控制性能分析 ===\n');

% 计算控制力
u = -x * K_new';

% 计算性能指标
theta_response = x(:,1);
settling_time_threshold = 0.02;  % 2%的稳定阈值

% 找出所有超出稳定带的时间点
unsettled_indices = find(abs(theta_response) >= settling_time_threshold * abs(x0(1)));

if ~isempty(unsettled_indices)
    % 找到最后一个超出稳定带的时间点
    last_unsettled_index = unsettled_indices(end);
    if last_unsettled_index < length(t)
        settling_time = t(last_unsettled_index + 1);
    else
        settling_time = t(end);
    end
else
    % 如果一开始就在稳定带内
    settling_time = 0;
end

% 超调量（对于初始条件响应，看最大偏离）
if x0(1) > 0
    % 正初始角度
    peak_overshoot = max(theta_response) - x0(1);
    overshoot_positive = (peak_overshoot / abs(x0(1))) * 100;
    
    peak_undershoot = min(theta_response);
    overshoot_negative = (abs(peak_undershoot) / abs(x0(1))) * 100;
    
    overshoot = max(overshoot_positive, overshoot_negative);
    
elseif x0(1) < 0
    % 负初始角度
    peak_overshoot = min(theta_response) - x0(1);
    overshoot_positive = (abs(peak_overshoot) / abs(x0(1))) * 100;
    
    peak_undershoot = max(theta_response);
    overshoot_negative = (peak_undershoot / abs(x0(1))) * 100;
    
    overshoot = max(overshoot_positive, overshoot_negative);
else
    overshoot = 0;
end

fprintf('初始条件响应性能:\n');
fprintf('  调整时间 (到2%%): %.3f 秒\n', settling_time);
fprintf('  最大超调量: %.2f%%\n', overshoot);
fprintf('  稳态误差: %.4f rad\n', theta_response(end));

% 绘制控制力
figure;
plot(t, u, 'r', 'LineWidth', 2);
title('控制力 u');
xlabel('时间 (s)'); ylabel('控制力 (N)');
grid on;

%% 6. 验证闭环稳定性
fprintf('\n=== 6. 稳定性验证 ===\n');

closed_loop_poles = eig(A_cl_new);
fprintf('闭环系统极点:\n');
disp(closed_loop_poles);

if all(real(closed_loop_poles) < 0)
    fprintf('闭环系统稳定！\n');
    stability_margin = min(abs(real(closed_loop_poles)));
    fprintf('稳定裕度: %.4f\n', stability_margin);
else
    fprintf('闭环系统不稳定！需要重新设计控制器。\n');
end

%% 7. 比较不同控制器
fprintf('\n=== 7. 控制器比较 ===\n');

% 使用您原来的极点配置方法，但用初始条件响应测试

% 配置1
s1 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
s2 = -zeta*wn - 1i*wn*sqrt(1-zeta^2);
P1 = [s1, s2, -10 - 0.0001*1i, -10 + 0.0001*1i];
K1 = place(A, B, P1);

% 配置2  
P2 = [s1, s2, -15 - 0.0001*1i, -15 + 0.0001*1i];
K2 = place(A, B, P2);

% 配置3
s1_highdamp = -0.7*wn + 1i*wn*sqrt(1-0.7^2);
s2_highdamp = -0.7*wn - 1i*wn*sqrt(1-0.7^2);
P3 = [s1_highdamp, s2_highdamp, -10 - 0.0001*1i, -10 + 0.0001*1i];
K3 = place(A, B, P3);

% 比较初始条件响应
sys1 = ss(A-B*K1, B, C, D);
sys2 = ss(A-B*K2, B, C, D);
sys3 = ss(A-B*K3, B, C, D);
sys_new = ss(A-B*K_new, B, C, D);

[y1, t, x1] = initial(sys1, x0, t);
[y2, t, x2] = initial(sys2, x0, t);
[y3, t, x3] = initial(sys3, x0, t);
[y_new, t, x_new] = initial(sys_new, x0, t);

figure;
plot(t, x1(:,1), 'r', t, x2(:,1), 'g--', t, x3(:,1), 'b-.', t, x_new(:,1), 'k-', 'LineWidth', 2);
title('不同控制器的角度响应对比');
xlabel('时间 (s)'); ylabel('角度 (rad)');
legend('配置1', '配置2', '配置3', '新配置', 'Location', 'best');
grid on;

%% 8. 保存控制器参数
fprintf('\n=== 8. 保存控制器参数 ===\n');

controller_params.K_new = K_new;
controller_params.K1 = K1;
controller_params.K2 = K2;
controller_params.K3 = K3;
controller_params.desired_poles = desired_poles;
controller_params.A = A;
controller_params.B = B;

save('improved_controller_params.mat', '-struct', 'controller_params');

fprintf('改进的控制器参数已保存\n');
fprintf('推荐使用 K_new 进行仿真测试\n');

fprintf('\n设计完成！现在使用初始条件响应来评估倒立摆控制性能。\n');