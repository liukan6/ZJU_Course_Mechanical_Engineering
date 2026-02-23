%% =========== 1) 读取 / 构建植物模型 P(s) ============
clear; close all; clc;
load("PhyPara.mat");
[Np, Dp] = computePlantFromPhyPara(PhyPara);
fprintf('Plant constructed from PhyPara.\n');

% Show plant
s = tf('s');
P = tf(Np, Dp);
disp('Plant P(s):'); P

%% =========== 2) 选择设计目标（期望闭环极点） ============
% 主导极点 -10 +/- j10，其他放在 -50
p_des = [ -10+10j, -10-10j, -50, -50];

%% =========== 3) 多项式配位求解 PID 参数 ============
% 把 proper PID 写成有理形式：
% C(s) = (b2 s^2 + b1 s + b0) / (tauD s^2 + s)
% 其中 b2 = Kp*tauD + Kd, b1 = Kp + Ki*tauD, b0 = Ki

% 构造期望多项式 A_des (系数 s^5 ... s^0)
A_des = poly(p_des);

% unknowns x = [b2,b1,b0,tauD]
opts = optimoptions('fsolve','Display','iter','TolFun',1e-12,'TolX',1e-12);
fun = @(x) pid_nonlinear_eqs(x, Np, Dp, A_des);
x0 = [1, 1, 1, 0.01]; % 初始猜测
xsol = fsolve(fun, x0, opts);
b2 = xsol(1); b1 = xsol(2); b0 = xsol(3); tauD = xsol(4);
Ki = b0;
Kp = b1 - Ki * tauD;
Kd = b2 - Kp * tauD;

fprintf('Design results:\n tauD = %.6g\n Kp = %.6g\n Ki = %.6g\n Kd = %.6g\n', tauD, Kp, Ki, Kd);
Nc = [b2 b1 b0];
Dc = [tauD, 1, 0];
save("FedPoly.mat","Nc","Dc");

%% =========== 4) 构造控制器与闭环，分析极点与时域指标 ============
C = Kp + Ki/s + (Kd*s)/(tauD*s + 1);
T = minreal(feedback(C*P, 1));           % reference -> alpha
U_from_R = minreal(C/(1+C*P));           % control input from reference

fprintf('\nClosed-loop poles:\n'); disp(pole(T));

% 阶跃响应
si = stepinfo(T);
disp('stepinfo(T):'); disp(si);

% 估算 dominant pair 的 zeta, wn, ts, OS
clp = pole(T);
% 取实部最大（最接近虚轴）的一对复数作为主导极点（保守选择）
[~, idxsort] = sort(real(clp),'descend');
% 尝试找复对
dom = [];
for k = 1:length(idxsort)
    if imag(clp(idxsort(k)))>0
        dom = clp(idxsort(k));
        break;
    end
end
if isempty(dom)
    % 没有复共轭主导对，选实部最大的作为dom
    dom = clp(idxsort(1));
end
wn = abs(dom);
zeta = -real(dom)/wn;
ts_approx = 4/(zeta*wn);
OS_approx = 100*exp(-zeta*pi/sqrt(1-zeta^2));
fprintf('Estimated dominant: zeta=%.4f, wn=%.4f, ts≈%.4f s, OS≈%.2f%%\n', zeta, wn, ts_approx, OS_approx);

%% =========== 5) 画图 ============
figure;
subplot(2,1,1); step(T); grid on; title('Closed-loop output (alpha) to unit step');
% subplot(2,1,2); step(U_from_R); grid on; title('Control input to unit step');
subplot(2,1,2); impulse(U_from_R); grid on; title('Control input to unit impulse');

%% =========== 辅助函数 ============
function F = pid_nonlinear_eqs(x, Np, Dp, A_des)
    % x = [b2,b1,b0,tauD]
    b2 = x(1); b1 = x(2); b0 = x(3); tauD = x(4);

    Dc = [tauD, 1, 0];
    A_cl = conv(Dp, Dc) + [0 0 conv(Np,[b2 b1 b0])];
    A_des_scaled = tauD * A_des;

    F = zeros(4,1);
    F(1) = A_cl(2) - A_des_scaled(2);
    F(2) = A_cl(3) - A_des_scaled(3);
    F(3) = A_cl(4) - A_des_scaled(4);
    F(4) = A_cl(5) - A_des_scaled(5);
end

function [num_full, den_full] = computePlantFromPhyPara(PhyPara)
    mp = PhyPara.mp; Lc = PhyPara.Lc; Jp = PhyPara.Jp; Bp = PhyPara.Bp;
    Lt = PhyPara.Lt; Jt = PhyPara.Jeq;Beq = PhyPara.Beq;
    Rm = PhyPara.Rm; kt = PhyPara.kt; km = PhyPara.km; Kg = PhyPara.Kg;
    eta_m = PhyPara.yitam; eta_g = PhyPara.yitag; g = PhyPara.Accg;
    Bt = (eta_m*eta_g*kt*km*Kg^2+Beq*Rm)/Rm;
    
    I_p = Jp + mp * Lc^2;
    I_t = Jt + mp * Lt^2;
    
    % ---- pendulum natural freq and damping ratio ----
    omega_r = sqrt(mp * Lc * g / I_p);
    zeta_r = Bp / (2 * I_p * omega_r);
    
    % ---- table (rotary arm) dynamics ----
    tau_t = I_t / Bt;
    omega_t = 1 / tau_t;
    
    % ---- motor/drive gains ----
    Ku = eta_m * eta_g * kt * Kg / Rm;
    Kbar_t = Ku / Bt;
    Kbar_a = (mp * Lt * Lc) / Ku;
    K_r = Lt / g;
    

    denom = 1 - Kbar_a * Kbar_t * K_r * omega_t * omega_r^2;
    if abs(denom) < 1e-12
        warning('Denominator in coefficient calculation is very small (near singular). Check parameters.');
    end
    

    a0 = - omega_t * omega_r^2 / denom;
    a1 = (2 * zeta_r * omega_r * omega_t - omega_r^2) / denom;
    a2 = (omega_t + 2 * zeta_r * omega_r) / denom;
    b2 = Kbar_t * K_r * omega_t * omega_r^2 / denom;

    num_full = [b2, 0];
    den_full = [1, a2, a1, a0];
end
