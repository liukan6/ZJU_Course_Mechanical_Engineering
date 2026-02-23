% 加载PID控制器仿真结果数据
load("B_CtrlResData_PID.mat");

% 创建图形窗口
figure;

% 1. 期望输出与实际响应对比
subplot(2,2,1);
plot(CtrlResData.time, CtrlResData.Yd, 'g-', 'LineWidth', 1.5); 
hold on;
plot(CtrlResData.time, CtrlResData.FedY, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('角度 (°)');
title('期望输出与实际响应对比');
legend('期望角度 Y_d', '实际响应 Y', 'Location', 'best');
grid on;

% 2. 控制输入信号
subplot(2,2,2);
plot(CtrlResData.time, CtrlResData.uout, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('控制量');
title('控制输入信号');
grid on;

% 3. 摆杆角度响应
subplot(2,2,3);
plot(CtrlResData.time, CtrlResData.alpha, 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\alpha (°)');
title('摆杆角度响应');
grid on;

% 4. 旋转臂角度响应
subplot(2,2,4);
plot(CtrlResData.time, CtrlResData.theta, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\theta (°)');
title('旋转臂角度响应');
grid on;

% 设置整体标题
sgtitle('PID控制器性能分析');