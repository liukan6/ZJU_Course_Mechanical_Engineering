dt=0.005;%采样时间
data=load ('极点配置后倒立摆数据.txt');
angle_data=(data(1:end,1)+180)/180*pi;
pos_data=data(1:end,2)/1000;
MotorAcc_data=data(1:end,5);
len=size(angle_data,1);
t=0:dt:dt*(len-1);

% subplot(3,1,1)
% plot(t,angle_data);legend('angele');
% subplot(3,1,2)
% plot(t,MotorAcc_data);legend('MotorAcc');
% subplot(3,1,3)
% plot(t,pos_data);legend('weizhi');
subplot(2,1,1)
yyaxis left
plot(t,angle_data);
yyaxis right
plot(t,MotorAcc_data);
legend('摆杆角度','加速度');
subplot(2,1,2)
plot(t,pos_data);legend('电机位置');