%% =========== 1) 读取 / 构建植物模型 P(s) ============
clear; close all; clc;
load("PhyPara.mat");
[Np, Dp] = computePlantFromPhyPara(PhyPara);
fprintf('Plant constructed from PhyPara.\n');

% Show plant
s = tf('s');
P = tf(Np, Dp);

for poleindex = 1:6
%% =========== 2) 选择设计目标（期望闭环极点） ============
omega_cl_list = [5 15 25 5 15 25];
omega_cl = omega_cl_list(poleindex);
zeta = 0.707;
real_part = -zeta * omega_cl;
imag_part = omega_cl * sqrt(1 - zeta^2);

p1 = real_part + imag_part * 1i;
p2 = real_part - imag_part * 1i;
p3 = -5*omega_cl;
p4 = -5*omega_cl;
p5 =-5*omega_cl;
p_des = [p1, p2, p3, p4, p5];

%% =========== 3) 多项式配位求解控制器参数 ============
% C(s) = (bc3 s^3 + bc2 s^2 + bc1 s + bc0) / (s^2 (s + ac0))

% 构造期望多项式 A_des (系数 s^5 ... s^0)
A_des = poly(p_des);

% unknowns x = [bc3,bc2,bc1,bc0,ac0]
opts = optimoptions('fsolve','Display','iter','TolFun',1e-12,'TolX',1e-12);
fun = @(x) pid_nonlinear_eqs(x, Np, Dp, A_des);
x0 = [1, 1, 1, 1, 0.01]; % 初始猜测
xsol = fsolve(fun, x0, opts);
bc3 = xsol(1); bc2 = xsol(2); bc1 = xsol(3); bc0 = xsol(4); ac0 = xsol(5);

Nc = [bc3 bc2 bc1 bc0];
Dc = [1 ac0 0 0];
save("Aufgabe_C\FedPoly_"+poleindex+".mat","Nc","Dc");



%% =========== 4) 构造控制器与闭环，分析极点与时域指标 ============
C = (bc3*s^3+bc2*s^2+bc1*s+bc0)/(s^2*(s+ac0));
T = minreal(feedback(C*P, 1));           % reference -> alpha
U_from_R = minreal(C/(1+C*P));           % control input from reference

fprintf('\nClosed-loop poles:\n'); disp(pole(T));

end

% % 阶跃响应
% si = stepinfo(T);
% disp('stepinfo(T):'); disp(si);
% 
% % 估算 dominant pair 的 zeta, wn, ts, OS
% clp = pole(T);
% % 取实部最大（最接近虚轴）的一对复数作为主导极点（保守选择）
% [~, idxsort] = sort(real(clp),'descend');
% % 尝试找复对
% dom = [];
% for k = 1:length(idxsort)
%     if imag(clp(idxsort(k)))>0
%         dom = clp(idxsort(k));
%         break;
%     end
% end
% if isempty(dom)
%     % 没有复共轭主导对，选实部最大的作为dom
%     dom = clp(idxsort(1));
% end
% wn = abs(dom);
% zeta = -real(dom)/wn;
% ts_approx = 4/(zeta*wn);
% OS_approx = 100*exp(-zeta*pi/sqrt(1-zeta^2));
% fprintf('Estimated dominant: zeta=%.4f, wn=%.4f, ts≈%.4f s, OS≈%.2f%%\n', zeta, wn, ts_approx, OS_approx);


%% =========== 5) 画图 ============
% figure;
% subplot(2,1,1); step(T); grid on; title('Closed-loop output (alpha) to unit step');
% % subplot(2,1,2); step(U_from_R); grid on; title('Control input to unit step');
% subplot(2,1,2); impulse(U_from_R); grid on; title('Control input to unit impulse');

%% =========== 辅助函数 ============
function F = pid_nonlinear_eqs(x, Np, Dp, A_des)
    % x = [bc3,bc2,bc1,bc0,ac0]
    bc3 = x(1); bc2 = x(2); bc1 = x(3); bc0 = x(4); ac0 = x(5);

    A_cl = conv(Dp, [1 ac0 0 0]) + [0 0 conv(Np,[bc3 bc2 bc1 bc0])];
    A_des_scaled = A_cl(1) * A_des;

    F = zeros(5,1);
    F(1) = A_cl(2) - A_des_scaled(2);
    F(2) = A_cl(3) - A_des_scaled(3);
    F(3) = A_cl(4) - A_des_scaled(4);
    F(4) = A_cl(5) - A_des_scaled(5);
    F(5) = A_cl(6) - A_des_scaled(6);
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
