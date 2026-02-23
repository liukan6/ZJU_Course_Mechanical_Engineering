clc;
clear;
load("systempara.mat");

numG=[0 0 m*l];
denG=[I + m*l^2, 0, -m*g*l];
[numGsD,denGsD]=tfdata(G_d);
sysG=feedback(G,1);
subplot(3,2,1);
step(sysG)
title('校正前系统阶跃响应');
subplot(3,2,2)
rlocus(G)
title('校正前系统根轨迹');
syms ksi;
ksi=solve(-ksi*pi/sqrt(1-ksi^2)==log(0.1),ksi);
ksi=vpa(ksi,2);
ksi=round(ksi,2);
Wn=round(4/ksi/0.5);
s1=-ksi*Wn+1i*Wn*sqrt(1-ksi^2);
s2=-ksi*Wn-1i*Wn*sqrt(1-ksi^2);
s3=-40;
s4=-40;
syms Kd Kp Ki td s;
RootEquation_desire=(s-s1)*(s-s2)*(s-s3)*(s-s4);
coeff_desire=coeffs(RootEquation_desire,s,'all');
RootEquation_Reality=(td*s^2+s)*(s^2-28.994)+2.96*((Kp*td+Kd)*s^2+(Ki*td+Kp)*s+Ki);
coeff_Reality=coeffs(RootEquation_Reality,s,'all');
coeff_Reality=coeff_Reality/td;
[td,Kp,Ki,Kd]=solve(coeff_desire==coeff_Reality,td,Kp,Ki,Kd);
Kp=double(Kp);
Ki=double(Ki);
Kd=double(Kd);
td=double(td);
numH=[Kp*td+Kd Ki*td+Kp Ki];
denH=[td 1 0];
sysH=tf(numH,denH);
[z,p,k]=tf2zp(numH,denH);
numGH=conv(numG,numH);
denGH=conv(denG,denH);

sysGH=series(sysG,sysH);
sys=feedback(sysGH,1);
syszpk=zpk(sys);
% [z,p,k]=zpkdata(syszpk,'v')
subplot(3,2,3);
step(sys)
title('校正后系统阶跃响应');
subplot(3,2,4);
rlocus(sysGH)
title('校正后系统根轨迹');
sysH=tf(numH,denH);
sysHD=c2d(sysH,0.005);
[numHD,denHD]=tfdata(sysHD);
s1=-30+11.3i;
s2=-30-11.3i;
s3=-100;
s4=-100;
syms Kd1 Kp1 Ki1 td1 s;
RootEquation_desire=(s-s1)*(s-s2)*(s-s3)*(s-s4);
coeff_desire=coeffs(RootEquation_desire,s,'all');
RootEquation_Reality=(td1*s^2+s)*(s^2-28.994)+2.96*((Kp1*td1+Kd1)*s^2+(Ki1*td1+Kp1)*s+Ki1);
coeff_Reality=coeffs(RootEquation_Reality,s,'all');
coeff_Reality=coeff_Reality/td1;
[td1,Kp1,Ki1,Kd1]=solve(coeff_desire==coeff_Reality,td1,Kp1,Ki1,Kd1);
Kp1=double(Kp1);
Ki1=double(Ki1);
Kd1=double(Kd1);
td1=double(td1);
numH1=[Kp1*td1+Kd1 Ki1*td1+Kp1 Ki1];
denH1=[td1 1 0];
sysH1=tf(numH1,denH1);
sysGH1=series(sysG,sysH1);
sys=feedback(sysGH1,1);
subplot(3,2,5);
step(sys)
title('第二次校正后系统阶跃响应');
subplot(3,2,6);
rlocus(sysGH)
title('第二次校正后系统根轨迹');
sysHD1=c2d(sysH1,0.005);
[numHD1,denHD1]=tfdata(sysHD1);