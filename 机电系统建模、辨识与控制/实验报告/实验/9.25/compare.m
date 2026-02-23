%%
clear;
clc;
close all;

%% 
dt=0.005;%采样时间
time=6;

load('模型参数.mat');
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



out=sim('clockwise_sim.slx');
residuals = out.angle-angle_data;

figure;
subplot(2,1,1);
plot(t,out.angle);
hold on;
plot(t,angle_data);
legend('angle\_sim', 'angle\_pra'); 

subplot(2,1,2)
plot(t,residuals);legend('residuals');