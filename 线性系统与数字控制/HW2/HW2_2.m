% Root Locus analysis for discrete-time system
clc; clear; close all;

% 定义开环传递函数 G(z) = K / [z(z - 0.2)(z - 0.4)]
z1 = 0;         % 极点1
z2 = 0.2;       % 极点2
z3 = 0.4;       % 极点3

% 开环传递函数（K 为可变参数，不包含在传递函数中）
num = 1; % 这里只是占位，K 不包含在传递函数中，用 root locus 会扫描 K
den = conv([1 -z1], conv([1 -z2], [1 -z3]));  % 乘起来得到 z(z-0.2)(z-0.4)

Ts = 1;  % 假设采样周期为1，单位样本系统
G = tf(num, den, Ts);  % 离散系统，不包含 K 增益

% ----------------------------
% 绘制根轨迹图
figure;
rlocus(G);
title('Root Locus of Open-loop Discrete-time System');
zgrid; % 加入单位圆、阻尼比等参考线
axis equal;

% ----------------------------
% 可选：找出所有稳定闭环极点对应的K范围（K > 0）
% 自动搜索单位圆内的根
Kvec = linspace(0, 200, 10000); % 尝试 K 从 0 到 200 的多个值
stable_K = [];

for i = 1:length(Kvec)
    poles_k = roots(den + Kvec(i) * [0 0 0 1]); % 特征方程：den + K*num = 0
    if all(abs(poles_k) < 1)  % 所有极点都在单位圆内
        stable_K(end+1) = Kvec(i); 
    end
end

if ~isempty(stable_K)
    fprintf('\n使系统稳定的 K 值范围大约是：\n');
    fprintf('K ∈ (%.4f, %.4f)\n', min(stable_K), max(stable_K));
else
    warning('在指定的K范围内没有找到使系统稳定的值');
end
