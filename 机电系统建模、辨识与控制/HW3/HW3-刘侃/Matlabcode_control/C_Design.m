%% =========== 1) 构建倒立摆系统模型 P(s) ============
clear; close all; clc;
load("PhyPara.mat");
[Np, Dp] = buildPlantModel(PhyPara);
fprintf('系统模型构建完成.\n');

% 建立传递函数
s = tf('s');
P = tf(Np, Dp);

% 六种设计工况循环
for case_id = 1:6
%% =========== 2) 设定闭环极点配置目标 ============
% 工况参数：带宽与初始角度
omega_cl_cases = [5 15 25 5 15 25];
alpha0_cases = [6 6 6 10 10 10];  % 度
omega_cl = omega_cl_cases(case_id);
alpha0_deg = alpha0_cases(case_id);
zeta_cl = 0.707;

% 计算主导极点
sigma_dom = -zeta_cl * omega_cl;
omega_dom = omega_cl * sqrt(1 - zeta_cl^2);
dom_pole1 = sigma_dom + omega_dom * 1i;
dom_pole2 = sigma_dom - omega_dom * 1i;

% 配置辅助极点（主导极点实部的5倍）
aux_pole_real = 5 * sigma_dom;
aux_pole1 = aux_pole_real;
aux_pole2 = aux_pole_real;
aux_pole3 = aux_pole_real;

% 期望极点集合
desired_poles = [dom_pole1, dom_pole2, aux_pole1, aux_pole2, aux_pole3];

%% =========== 3) 基于多项式匹配的控制器参数求解 ============
% 控制器结构: C(s) = (bc3*s^3 + bc2*s^2 + bc1*s + bc0) / (s^2*(s + ac0))

% 构建期望特征多项式
A_desired = poly(desired_poles);

% 优化求解控制器参数: x = [bc3, bc2, bc1, bc0, ac0]
options = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-10, 'TolX', 1e-10);
eqn_system = @(x) controller_eqns(x, Np, Dp, A_desired);
initial_guess = [1, 1, 1, 1, 0.1];
solution = fsolve(eqn_system, initial_guess, options);

bc3 = solution(1); bc2 = solution(2); bc1 = solution(3); 
bc0 = solution(4); ac0 = solution(5);

% 保存控制器参数
Nc = [bc3 bc2 bc1 bc0];
Dc = [1 ac0 0 0];
save("C\FedPoly_" + case_id + ".mat", "Nc", "Dc");

%% =========== 4) 构建控制系统并分析性能 ============
% 控制器传递函数
C = (bc3*s^3 + bc2*s^2 + bc1*s + bc0) / (s^2*(s + ac0));

% 闭环系统
T_cl = minreal(feedback(C * P, 1));        % 参考输入到摆角
U_cl = minreal(C / (1 + C * P));           % 参考输入到控制量

% 显示闭环极点
fprintf('\n工况 %d 闭环极点:\n', case_id); 
disp(pole(T_cl));

% 性能指标计算
cl_poles = pole(T_cl);
[~, idx] = sort(real(cl_poles), 'descend');
dominant_pole = [];
for k = 1:length(idx)
    if imag(cl_poles(idx(k))) > 0
        dominant_pole = cl_poles(idx(k));
        break;
    end
end

if ~isempty(dominant_pole)
    wn_est = abs(dominant_pole);
    zeta_est = -real(dominant_pole) / wn_est;
    ts_est = 4 / (zeta_est * wn_est);
    OS_est = 100 * exp(-zeta_est * pi / sqrt(1 - zeta_est^2));
    
    fprintf('性能估计: 阻尼比=%.3f, 自然频率=%.3f rad/s\n', zeta_est, wn_est);
    fprintf('          调节时间=%.3f s, 超调量=%.1f%%\n', ts_est, OS_est);
end

end

%% =========== 辅助函数：控制器方程求解 ============
function F = controller_eqns(x, Np, Dp, A_des)
    % 控制器参数提取
    bc3 = x(1); bc2 = x(2); bc1 = x(3); bc0 = x(4); ac0 = x(5);

    % 计算实际闭环特征多项式
    % A_cl(s) = Dp(s)*Dc(s) + Np(s)*Nc(s)
    A_actual = conv(Dp, [1 ac0 0 0]) + [0 0 conv(Np, [bc3 bc2 bc1 bc0])];
    
    % 多项式系数匹配（归一化处理）
    A_des_scaled = A_des * A_actual(1) / A_des(1);
    
    % 构建误差向量（匹配2~6阶系数）
    F = zeros(5, 1);
    for k = 1:5
        F(k) = A_actual(k+1) - A_des_scaled(k+1);
    end
end

%% =========== 辅助函数：系统模型构建 ============
function [num, den] = buildPlantModel(para)
    % 参数提取
    mp = para.mp; Lc = para.Lc; Jp = para.Jp; Bp = para.Bp;
    Lt = para.Lt; Jt = para.Jeq; Beq = para.Beq;
    Rm = para.Rm; kt = para.kt; km = para.km; Kg = para.Kg;
    eta_m = para.yitam; eta_g = para.yitag; g = para.Accg;
    
    % 等效阻尼计算
    Bt = (eta_m * eta_g * kt * km * Kg^2 + Beq * Rm) / Rm;
    
    % 转动惯量计算
    I_p = Jp + mp * Lc^2;      % 摆杆总惯量
    I_t = Jt + mp * Lt^2;      % 旋转臂总惯量
    
    % 摆杆动态参数
    omega_p = sqrt(mp * Lc * g / I_p);      % 自然频率
    zeta_p = Bp / (2 * I_p * omega_p);      % 阻尼比
    
    % 旋转臂动态参数  
    tau_t = I_t / Bt;                       % 时间常数
    omega_t = 1 / tau_t;                    % 特征频率
    
    % 系统增益计算
    Ku = eta_m * eta_g * kt * Kg / Rm;      % 电机增益
    Kbar_t = Ku / Bt;                       % 等效增益
    Kbar_a = (mp * Lt * Lc) / Ku;           % 耦合增益
    K_r = Lt / g;                           % 几何增益
    
    % 系统系数计算
    denominator = 1 - Kbar_a * Kbar_t * K_r * omega_t * omega_p^2;
    
    if abs(denominator) < 1e-12
        warning('系统参数接近奇异，请检查参数合理性');
    end
    
    % 传递函数系数
    a0 = -omega_t * omega_p^2 / denominator;
    a1 = (2 * zeta_p * omega_p * omega_t - omega_p^2) / denominator;
    a2 = (omega_t + 2 * zeta_p * omega_p) / denominator;
    b2 = Kbar_t * K_r * omega_t * omega_p^2 / denominator;

    num = [b2, 0];      % 分子多项式
    den = [1, a2, a1, a0];  % 分母多项式
end