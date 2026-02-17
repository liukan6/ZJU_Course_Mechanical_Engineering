%% 定义系统
clear;clc;

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
D=zeros(3,1);

sysc=ss(A,B,C,D); % 连续时间系统
transfc=tf(sysc); % 连续时间传递函数
Ts=0.1; % 采样周期
sysd=c2d(sysc,Ts);% 离散时间系统
transfd=tf(sysd); % 离散时间传递函数

save("state_space.mat","sysc","sysd","Ts");

%% 计算特征值
eigenvalues = eig(sysc);

% 创建图形窗口
figure;

% 在复平面上绘制特征值（实部 vs 虚部）
plot(real(eigenvalues), imag(eigenvalues), 'o', ...
     'MarkerSize', 4, ...
     'MarkerFaceColor', 'red', ...
     'MarkerEdgeColor', 'black');

% 添加参考线
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';

% 添加标签和标题
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Eigenvalues of Matrix A in Complex Plane');
grid on;
box on;

% 调整坐标轴范围以适应数据
axis equal;
x_range = max(abs(real(eigenvalues))) * 1.2;
y_range = max(abs(imag(eigenvalues))) * 1.2;
if x_range == 0, x_range = 1; end
if y_range == 0, y_range = 1; end
xlim([-x_range, x_range]);
ylim([-y_range, y_range]);

% 显示特征值
disp('Computed eigenvalues:');
disp(eigenvalues);

%% 计算能控性判别矩阵
Qc = ctrb(sysd);
Qo = obsv(sysd);

% 显示结果
disp('能控性判别矩阵 Qc:');
disp(Qc);
disp(['秩 = ', num2str(rank(Qc))]);

disp('能观性判别矩阵 Qo:');
disp(Qo);
disp(['秩 = ', num2str(rank(Qo))]);