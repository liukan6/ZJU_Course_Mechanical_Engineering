function optimalPID = invertedPendulumPIDtuner()
%% 多变量倒立摆PID控制器优化程序
% 该程序同时优化小车位置和两个摆杆角度的控制
% 输出: optimalPID - 包含最优PID参数的结构体

%% 1. 系统定义
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

%% 2. 离散化
Ts = 0.1;  % 采样时间 (秒)
sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts, 'zoh');  % 零阶保持法离散化
G = sysd.A; 
H = sysd.B; 
Cd = sysd.C; 
Dd = sysd.D;

%% 3. 多变量PID控制器结构设计
% 我们将使用三个独立的PID控制器：
% PID1: 控制小车位置x
% PID2: 控制摆杆角度φ
% PID3: 控制摆杆角度θ
% 总控制输入 u = u1 + u2 + u3

% 参考信号设置
ref.x = 0;      % 小车位置参考 (最终目标为0)
ref.phi = 0;    % 摆杆角度φ参考 (垂直向上)
ref.theta = 0;  % 摆杆角度θ参考 (垂直向上)

%% 4. 自动PID参数整定
fprintf('开始多变量PID参数优化...\n');
fprintf('--------------------------------------\n');

% 使用粒子群优化算法寻找最优参数
options = optimoptions('particleswarm', ...
    'SwarmSize', 40, ...
    'MaxIterations', 100, ...
    'Display', 'iter', ...
    'UseParallel', true);

% 参数边界 [Kp_x, Ki_x, Kd_x, Kp_phi, Ki_phi, Kd_phi, Kp_theta, Ki_theta, Kd_theta]
lb = [0.1, 0, 0.1,  1.0, 0, 0.5,  1.0, 0, 0.5];  % 下限
ub = [5.0, 0, 3.0,   10.0, 0, 8.0,  10.0, 0, 8.0]; % 上限

% 运行优化
optimal_params = particleswarm(@(params) multiPIDcost(params, sysd, Ts, ref), 9, lb, ub, options);

% 提取最优参数
optimalPID.x = struct('Kp', optimal_params(1), 'Ki', optimal_params(2), 'Kd', optimal_params(3));
optimalPID.phi = struct('Kp', optimal_params(4), 'Ki', optimal_params(5), 'Kd', optimal_params(6));
optimalPID.theta = struct('Kp', optimal_params(7), 'Ki', optimal_params(8), 'Kd', optimal_params(9));

fprintf('\n--------------------------------------\n');
fprintf('优化完成! 最优PID参数:\n');
fprintf('小车位置PID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', optimalPID.x.Kp, optimalPID.x.Ki, optimalPID.x.Kd);
fprintf('摆杆角度φ PID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', optimalPID.phi.Kp, optimalPID.phi.Ki, optimalPID.phi.Kd);
fprintf('摆杆角度θ PID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', optimalPID.theta.Kp, optimalPID.theta.Ki, optimalPID.theta.Kd);
fprintf('--------------------------------------\n');

%% 5. 使用最优参数进行验证
verifyMultiPIDperformance(sysd, Ts, ref, optimalPID);

%% 成本函数 - 评估多变量PID性能
function cost = multiPIDcost(params, sys, Ts, ref)
    % 提取PID参数
    Kp_x = params(1); Ki_x = params(2); Kd_x = params(3);
    Kp_phi = params(4); Ki_phi = params(5); Kd_phi = params(6);
    Kp_theta = params(7); Ki_theta = params(8); Kd_theta = params(9);
    
    % 设置仿真参数
    Tf = 10;         % 仿真总时间 (秒)
    t_sim = 0:Ts:Tf; % 时间向量
    N = length(t_sim); % 仿真步数
    
    % 初始化变量
    x = zeros(size(sys.A, 1), 1); % 状态向量
    
    % 输出记录
    y_x = zeros(N,1);     % 小车位置
    y_phi = zeros(N,1);   % 摆杆角度φ
    y_theta = zeros(N,1); % 摆杆角度θ
    u_total = zeros(N,1); % 总控制输入
    
    % 误差记录
    e_x = zeros(N,1);
    e_phi = zeros(N,1);
    e_theta = zeros(N,1);
    
    % 积分项
    int_x = 0;
    int_phi = 0;
    int_theta = 0;
    
    % 前一时刻误差
    e_x_prev = 0;
    e_phi_prev = 0.1;
    e_theta_prev = 0.1;
    
    % 角度约束违反代价
    cost_angle = 0;
    
    % 仿真循环
    for k = 1:N
        % 获取当前输出
        y = sys.C * x;
        y_x(k) = y(1);
        y_phi(k) = y(2);
        y_theta(k) = y(3);
        
        % 计算误差
        e_x(k) = ref.x - y_x(k);
        e_phi(k) = ref.phi - y_phi(k);
        e_theta(k) = ref.theta - y_theta(k);
        
        % 计算各PID控制器的输出
        % 小车位置PID
        int_x = int_x + e_x(k);
        deriv_x = (e_x(k) - e_x_prev)/Ts;
        u_x = Kp_x * e_x(k) + Ki_x * Ts * int_x + Kd_x * deriv_x;
        
        % 摆杆角度φ PID
        int_phi = int_phi + e_phi(k);
        deriv_phi = (e_phi(k) - e_phi_prev)/Ts;
        u_phi = Kp_phi * e_phi(k) + Ki_phi * Ts * int_phi + Kd_phi * deriv_phi;
        
        % 摆杆角度θ PID
        int_theta = int_theta + e_theta(k);
        deriv_theta = (e_theta(k) - e_theta_prev)/Ts;
        u_theta = Kp_theta * e_theta(k) + Ki_theta * Ts * int_theta + Kd_theta * deriv_theta;
        
        % 总控制输入
        u_total(k) = u_x + u_phi + u_theta;
        
        % 控制输入限幅 (±15N)
        u_total(k) = max(min(u_total(k), 15), -15);
        
        % 更新系统状态
        x = sys.A * x + sys.B * u_total(k);
        
        % 检查摆杆角度约束 (超过20°视为违规)
        if abs(y_phi(k)) > deg2rad(20)
            cost_angle = cost_angle + 100 * (abs(y_phi(k)) - deg2rad(20))^2;
        end
        if abs(y_theta(k)) > deg2rad(20)
            cost_angle = cost_angle + 100 * (abs(y_theta(k)) - deg2rad(20))^2;
        end
        
        % 更新前一时刻误差
        e_x_prev = e_x(k);
        e_phi_prev = e_phi(k);
        e_theta_prev = e_theta(k);
    end
    
    % 计算性能指标
    % 1. 小车位置误差积分 (IAE)
    IAE_x = sum(abs(e_x)) * Ts;
    
    % 2. 角度误差积分 (IAE)
    IAE_phi = sum(abs(e_phi)) * Ts;
    IAE_theta = sum(abs(e_theta)) * Ts;
    
    % 3. 角度最大偏差
    max_phi = max(abs(y_phi));
    max_theta = max(abs(y_theta));
    
    % 4. 控制能量
    control_energy = sum(u_total.^2) * Ts;
    
    % 综合成本函数
    angle_cost = 5 * (IAE_phi + IAE_theta) + 10 * (max_phi + max_theta);
    position_cost = 1 * IAE_x;
    energy_cost = 0.01 * control_energy;
    
    cost = angle_cost + position_cost + energy_cost + cost_angle;
    
    % 系统不稳定惩罚
    if any(isnan(y_x)) || any(isinf(y_x)) || max(abs([y_phi; y_theta])) > deg2rad(45)
        cost = 1e6;
    end
end

%% 验证多变量PID性能
function verifyMultiPIDperformance(sys, Ts, ref, PIDparams)
    % 设置仿真参数
    Tf = 10;         % 仿真总时间 (秒)
    t_sim = 0:Ts:Tf; % 时间向量
    N = length(t_sim); % 仿真步数
    
    % 初始化变量
    x = zeros(size(sys.A, 1), 1); % 状态向量
    
    % 输出记录
    y_x = zeros(N,1);     % 小车位置
    y_phi = zeros(N,1);   % 摆杆角度φ
    y_theta = zeros(N,1); % 摆杆角度θ
    u_total = zeros(N,1); % 总控制输入
    u_components = zeros(N,3); % 各PID分量
    
    % 积分项
    int_x = 0;
    int_phi = 0;
    int_theta = 0;
    
    % 前一时刻误差
    e_x_prev = 0;
    e_phi_prev = 0;
    e_theta_prev = 1e-10;
    
    % 仿真循环
    for k = 1:N
        % 获取当前输出
        y = sys.C * x;
        y_x(k) = y(1);
        y_phi(k) = y(2);
        y_theta(k) = y(3);
        
        % 计算误差
        e_x = ref.x - y_x(k);
        e_phi = ref.phi - y_phi(k);
        e_theta = ref.theta - y_theta(k);
        
        % 计算各PID控制器的输出
        % 小车位置PID
        int_x = int_x + e_x;
        deriv_x = (e_x - e_x_prev)/Ts;
        u_x = PIDparams.x.Kp * e_x + PIDparams.x.Ki * Ts * int_x + PIDparams.x.Kd * deriv_x;
        
        % 摆杆角度φ PID
        int_phi = int_phi + e_phi;
        deriv_phi = (e_phi - e_phi_prev)/Ts;
        u_phi = PIDparams.phi.Kp * e_phi + PIDparams.phi.Ki * Ts * int_phi + PIDparams.phi.Kd * deriv_phi;
        
        % 摆杆角度θ PID
        int_theta = int_theta + e_theta;
        deriv_theta = (e_theta - e_theta_prev)/Ts;
        u_theta = PIDparams.theta.Kp * e_theta + PIDparams.theta.Ki * Ts * int_theta + PIDparams.theta.Kd * deriv_theta;
        
        % 总控制输入
        u_total(k) = u_x + u_phi + u_theta;
        u_components(k,:) = [u_x, u_phi, u_theta];
        
        % 控制输入限幅 (±15N)
        u_total(k) = max(min(u_total(k), 15), -15);
        
        % 更新系统状态
        x = sys.A * x + sys.B * u_total(k);
        
        % 更新前一时刻误差
        e_x_prev = e_x;
        e_phi_prev = e_phi;
        e_theta_prev = e_theta;
    end
    
    % 性能评估
    perf = struct();
    perf.IAE_x = sum(abs(ref.x - y_x)) * Ts;
    perf.IAE_phi = sum(abs(ref.phi - y_phi)) * Ts;
    perf.IAE_theta = sum(abs(ref.theta - y_theta)) * Ts;
    perf.max_phi = max(abs(y_phi));
    perf.max_theta = max(abs(y_theta));
    perf.control_energy = sum(u_total.^2) * Ts;
    
    % 绘图
    figure('Name', '多变量PID控制性能验证', 'Position', [100, 100, 1200, 1000]);
    
    % 小车位置
    subplot(3,2,1);
    stairs(t_sim, y_x, 'b', 'LineWidth', 1.5);
    hold on;
    plot(t_sim, ref.x * ones(size(t_sim)), 'r--', 'LineWidth', 1.5);
    title('小车位置响应');
    xlabel('时间 (秒)'); ylabel('位置 (m)');
    legend('实际位置', '参考位置');
    grid on;
    
    % 摆杆角度φ
    subplot(3,2,2);
    stairs(t_sim, rad2deg(y_phi), 'r', 'LineWidth', 1.5);
    hold on;
    plot(t_sim, rad2deg(ref.phi) * ones(size(t_sim)), 'r--', 'LineWidth', 1.5);
    title('摆杆角度 \phi');
    xlabel('时间 (秒)'); ylabel('角度 (°)');
    ylim([-20, 20]);
    grid on;
    
    % 摆杆角度θ
    subplot(3,2,3);
    stairs(t_sim, rad2deg(y_theta), 'g', 'LineWidth', 1.5);
    hold on;
    plot(t_sim, rad2deg(ref.theta) * ones(size(t_sim)), 'g--', 'LineWidth', 1.5);
    title('摆杆角度 \theta');
    xlabel('时间 (秒)'); ylabel('角度 (°)');
    ylim([-20, 20]);
    grid on;
    
    % 总控制输入
    subplot(3,2,4);
    stairs(t_sim, u_total, 'm', 'LineWidth', 1.5);
    title('总控制输入');
    xlabel('时间 (秒)'); ylabel('控制力 (N)');
    grid on;
    
    % 控制输入分量
    subplot(3,2,5);
    plot(t_sim, u_components(:,1), 'b', 'LineWidth', 1.5);
    hold on;
    plot(t_sim, u_components(:,2), 'r', 'LineWidth', 1.5);
    plot(t_sim, u_components(:,3), 'g', 'LineWidth', 1.5);
    title('控制输入分量');
    xlabel('时间 (秒)'); ylabel('控制力分量 (N)');
    legend('位置控制', '\phi控制', '\theta控制');
    grid on;
    
    % 性能指标
    subplot(3,2,6);
    axis off;
    text(0.1, 0.9, '性能指标:', 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.75, sprintf('小车位置IAE: %.4f m·s', perf.IAE_x), 'FontSize', 11);
    text(0.1, 0.60, sprintf('φ角度IAE: %.4f rad·s', perf.IAE_phi), 'FontSize', 11);
    text(0.1, 0.45, sprintf('θ角度IAE: %.4f rad·s', perf.IAE_theta), 'FontSize', 11);
    text(0.1, 0.30, sprintf('最大φ角度: %.2f°', rad2deg(perf.max_phi)), 'FontSize', 11);
    text(0.1, 0.15, sprintf('最大θ角度: %.2f°', rad2deg(perf.max_theta)), 'FontSize', 11);
    
    % 显示PID参数
    annotation('textbox', [0.6, 0.02, 0.3, 0.1], 'String', ...
        sprintf(['PID参数:\n' ...
                 '位置: Kp=%.3f, Ki=%.3f, Kd=%.3f\n' ...
                 'φ角度: Kp=%.3f, Ki=%.3f, Kd=%.3f\n' ...
                 'θ角度: Kp=%.3f, Ki=%.3f, Kd=%.3f'], ...
                PIDparams.x.Kp, PIDparams.x.Ki, PIDparams.x.Kd, ...
                PIDparams.phi.Kp, PIDparams.phi.Ki, PIDparams.phi.Kd, ...
                PIDparams.theta.Kp, PIDparams.theta.Ki, PIDparams.theta.Kd), ...
        'FontSize', 10, 'BackgroundColor', 'white');
end

end