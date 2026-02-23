%% 滤波截止频率影响研究 —— 顺摆辨识
clear; clc; close all;

%% 参数设置
taos = 0.005;      
zeta = 0.9;        
dt    = 0.005;     
tdown = 1;         
tup   = 5;         
time  = 6;         

% 截止频率列表
Wf_list = logspace(-2,2,5);

%% 数据读取
data = load('实测数据.txt');
angle_data = (data(1:time/dt+1,1)+180)/180*pi;  % rad
pos_data   = data(1:time/dt+1,2)/1000;          % m
MotorAcc_data = data(1:time/dt+1,5);

len = size(angle_data,1);
t = 0:dt:dt*(len-1);

% 转为时间信号矩阵
MotorAcc = [t', MotorAcc_data];
angle    = [t', angle_data];
pos      = [t', pos_data];

% 辨识段索引
kstart = tdown/dt + 1;
kend   = tup/dt + 1;
nseg   = kend - kstart + 1;

%% 初始化存储
Nw = length(Wf_list);
thetas = zeros(3, Nw);
RMSEs  = zeros(1, Nw);
fitPct = zeros(1, Nw);
residuals_all = cell(1, Nw);
angle_sims = zeros(len, Nw);

%% 循环测试不同 Wf
for i = 1:Nw
    Wf = Wf_list(i);
    fprintf('正在处理 Wf = %.3f rad/s  (%d/%d)\n', Wf, i, Nw);

    % 设置模型参数
    set_param('Filter','SimulationCommand','update'); % 如果模型已打开可刷新参数
    % Simulink 模型需要能从工作区直接读取变量 Wf, taos, zeta, dt 等

    % 滤波仿真
    out = sim('Filter.slx','SrcWorkspace','current'); 

    % 提取滤波结果用于辨识
    MotorAcc_F = zeros(1,nseg);
    angle_F    = zeros(1,nseg);
    angledot_F = zeros(1,nseg);
    angleddot_F= zeros(1,nseg);

    for idx = 1:nseg
        k = kstart + idx - 1;
        MotorAcc_F(idx)  = -out.Yt(k);
        angle_F(idx)     = out.x1(k);
        angledot_F(idx)  = out.x2(k);
        angleddot_F(idx) = out.x3(k);
    end

    % 最小二乘辨识
    phi_M = [angleddot_F' angledot_F' angle_F'];
    theta = (phi_M'*phi_M)\(phi_M'*MotorAcc_F');
    thetas(:,i) = theta;

    % 生成系统传递函数并仿真
    Gs = tf(-1,[theta(1) theta(2) theta(3)]);
    assignin('base','Gs',Gs); % 如果 Sim.slx 需要，可暂时保留这一句，否则删除
    outSim = sim('Sim.slx','SrcWorkspace','current');
    angle_sims(:,i) = outSim.angle;

    % 残差
    residuals = outSim.angle - angle_data;
    residuals_all{i} = residuals;
    RMSEs(i) = sqrt(mean(residuals.^2));
    fitPct(i) = 100 * (1 - norm(residuals) / norm(angle_data - mean(angle_data)));
end

%% 绘图
figure('Name','所有 Wf 的时域角度对比');
plot(t, angle_data, 'k', 'LineWidth', 1.5); hold on;
for i = 1:Nw
    plot(t, angle_sims(:,i), 'LineWidth', 1.2);
end
legend(['实测角度', compose('Wf=%.2f', Wf_list)], 'Location','best');
xlabel('时间 (s)');
ylabel('角度 (rad)');
title('不同滤波截止频率 Wf 的辨识结果时域对比');
grid on;

%% RMSE 与拟合度曲线
figure('Name','辨识性能指标随 Wf 变化');
subplot(2,1,1);
semilogx(Wf_list, RMSEs, '-o', 'LineWidth', 1.3);
ylabel('RMSE');
title('RMSE 随截止频率变化');
grid on;

subplot(2,1,2);
semilogx(Wf_list, fitPct, '-o', 'LineWidth', 1.3);
xlabel('Wf (rad/s)');
ylabel('拟合度 (%)');
title('拟合度随截止频率变化');
grid on;

%% 输出辨识参数
disp('各 Wf 的辨识参数 theta = [θ1 θ2 θ3]:');
disp(array2table(thetas','VariableNames',{'θ1','θ2','θ3'},'RowNames',compose('Wf=%.2f',Wf_list)));
