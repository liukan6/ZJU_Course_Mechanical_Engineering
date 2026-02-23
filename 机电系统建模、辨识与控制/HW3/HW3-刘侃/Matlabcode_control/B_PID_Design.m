%% =========== 1) 系统建模 ============
clear; close all; clc;
load("PhyPara.mat");
[plantNum, plantDen] = buildPlantModel(PhyPara);
fprintf('系统模型构建完成\n');

s = tf('s');
P = tf(plantNum, plantDen);
disp('被控对象 P(s):'); P

%% =========== 2) 期望极点配置 ============
desiredPoles = [-10+10j, -10-10j, -50, -50];

%% =========== 3) PID参数计算 ============
targetPoly = poly(desiredPoles);

options = optimoptions('fsolve','Display','iter','TolFun',1e-12,'TolX',1e-12);
eqSystem = @(x) solvePIDEquations(x, plantNum, plantDen, targetPoly);
initialGuess = [1, 1, 1, 0.01];
solution = fsolve(eqSystem, initialGuess, options);

b2 = solution(1); b1 = solution(2); b0 = solution(3); tauD = solution(4);
Ki = b0;
Kp = b1 - Ki * tauD;
Kd = b2 - Kp * tauD;

fprintf('PID参数计算结果:\n τD = %.6g\n Kp = %.6g\n Ki = %.6g\n Kd = %.6g\n', tauD, Kp, Ki, Kd);

%% =========== 4) 性能验证 ============
C = Kp + Ki/s + (Kd*s)/(tauD*s + 1);
closedLoopTF = minreal(feedback(C*P, 1));
controlInputTF = minreal(C/(1+C*P));

fprintf('\n闭环系统极点:\n'); disp(pole(closedLoopTF));

stepInfo = stepinfo(closedLoopTF);
disp('阶跃响应特性:'); disp(stepInfo);

[zeta, wn, settlingTime, overshoot] = estimatePerformance(closedLoopTF);
fprintf('性能估计: 阻尼比=%.4f, 自然频率=%.4f, 调节时间≈%.4f s, 超调≈%.2f%%\n', ...
        zeta, wn, settlingTime, overshoot);

%% =========== 5) 结果可视化 ============
figure;
subplot(2,1,1); 
step(closedLoopTF); grid on; 
title('闭环系统阶跃响应 (摆杆角度)');
subplot(2,1,2); 
impulse(controlInputTF); grid on; 
title('控制输入脉冲响应');

%% =========== 辅助函数 ============
function F = solvePIDEquations(x, numPlant, denPlant, targetPoly)
    b2 = x(1); b1 = x(2); b0 = x(3); tauD = x(4);
    
    denController = [tauD, 1, 0];
    actualPoly = conv(denPlant, denController) + [0 0 conv(numPlant,[b2 b1 b0])];
    scaledTarget = tauD * targetPoly;
    
    F = zeros(4,1);
    F(1) = actualPoly(2) - scaledTarget(2);
    F(2) = actualPoly(3) - scaledTarget(3);
    F(3) = actualPoly(4) - scaledTarget(4);
    F(4) = actualPoly(5) - scaledTarget(5);
end

function [num, den] = buildPlantModel(params)
    % 参数提取
    mp = params.mp; Lc = params.Lc; Jp = params.Jp; Bp = params.Bp;
    Lt = params.Lt; Jt = params.Jeq; Beq = params.Beq;
    Rm = params.Rm; kt = params.kt; km = params.km; Kg = params.Kg;
    eta_m = params.yitam; eta_g = params.yitag; g = params.Accg;
    
    % 等效阻尼计算
    Bt = (eta_m*eta_g*kt*km*Kg^2 + Beq*Rm)/Rm;
    
    % 转动惯量计算
    Ip = Jp + mp * Lc^2;
    It = Jt + mp * Lt^2;
    
    % 摆杆动态特性
    omega_p = sqrt(mp * Lc * g / Ip);
    zeta_p = Bp / (2 * Ip * omega_p);
    
    % 旋转臂动态特性
    tau_t = It / Bt;
    omega_t = 1 / tau_t;
    
    % 系统增益计算
    Ku = eta_m * eta_g * kt * Kg / Rm;
    Kt_bar = Ku / Bt;
    Ka_bar = (mp * Lt * Lc) / Ku;
    Kr = Lt / g;
    
    % 传递函数系数计算
    denominator = 1 - Ka_bar * Kt_bar * Kr * omega_t * omega_p^2;
    
    a0 = -omega_t * omega_p^2 / denominator;
    a1 = (2 * zeta_p * omega_p * omega_t - omega_p^2) / denominator;
    a2 = (omega_t + 2 * zeta_p * omega_p) / denominator;
    b2 = Kt_bar * Kr * omega_t * omega_p^2 / denominator;

    num = [b2, 0];
    den = [1, a2, a1, a0];
end

function [zeta, wn, ts, os] = estimatePerformance(sys)
    poles = pole(sys);
    
    % 寻找主导极点（实部最大的复共轭对）
    [~, idx] = sort(real(poles), 'descend');
    dominantPole = [];
    
    for i = 1:length(idx)
        if imag(poles(idx(i))) > 0
            dominantPole = poles(idx(i));
            break;
        end
    end
    
    if isempty(dominantPole)
        dominantPole = poles(idx(1));
    end
    
    wn = abs(dominantPole);
    zeta = -real(dominantPole) / wn;
    ts = 4 / (zeta * wn);
    os = 100 * exp(-zeta * pi / sqrt(1 - zeta^2));
end