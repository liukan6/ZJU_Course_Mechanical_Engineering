%% 辨识
clear;
clc;
close all;

%% 
% data=load ('实测数据.txt');
taos=0.005;%角频率
zeta=0.9;%角阻尼
Wf=10;
dt=0.005;%采样时间
tdown=1;
tup=5;
time=6;

data=load ('实测数据.txt');
angle_data=(data(1:time/dt+1,1)+180)/180*pi;
pos_data=data(1:time/dt+1,2)/1000;
MotorAcc_data=data(1:time/dt+1,5);
len=size(angle_data,1);
t=0:dt:dt*(len-1);

figure;
subplot(3,1,1)
plot(t,angle_data);legend('angele');
subplot(3,1,2)
plot(t,MotorAcc_data);legend('MotorAcc');
subplot(3,1,3)
plot(t,pos_data);legend('pos');

%设位置、角度、电机加速度为时间矩阵            
MotorAcc=[t',MotorAcc_data];
angle=[t',angle_data];
pos=[t',pos_data];

%打开滤波文件
out=sim('Filter.slx');

%取出一段数据进行辨识，tup不能超过（len-1）*0.005
for k=tdown/dt+1:tup/dt+1
    MotorAcc_F(k-tdown/dt) = -out.Yt(k);%加速度滤波得到的加速度
    angle_F(k-tdown/dt) = out.x1(k);%滤波之后的角度
    angledot_F(k-tdown/dt) = out.x2(k);%滤波之后的角速度
    angleddot_F(k-tdown/dt) = out.x3(k);%滤波之后的角加速度
end
% simulink模型的最小二乘法公式
phi_M = [angleddot_F' angledot_F' angle_F'];
theta = inv(phi_M'*phi_M)*phi_M'*MotorAcc_F';  %三个参数的辨识结果
Gs=tf(-1,[theta(1) theta(2) theta(3)]);

%% 比较
out=sim('Sim.slx');
residuals = out.angle-angle_data;

figure;
subplot(2,1,1);
plot(t,out.angle);
hold on;
plot(t,angle_data);
legend('angle\_sim', 'angle\_pra'); 

subplot(2,1,2)
plot(t,residuals);legend('residuals');