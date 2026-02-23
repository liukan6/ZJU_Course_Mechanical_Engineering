## A. equations of motion (EOM)

### Modeling of Inverted Pendulum

**Electrical circuit**

$$L_m\frac{dI_m}{dt} + R_m I_m + k_m \omega_m = V_m$$

Ignore the inductance of motor: $$R_m I_m + k_m \omega_m = V_m$$

$$\tau_m = \eta_m k_t\, I_m$$

**Motor shaft**

$$J_m\dot\omega_m + B_m \omega_m + \tau_{ml} = \tau_m$$

Ignore the moment of inertia and the viscous damping coefficient of motor: $$\tau_{ml} = \tau_m$$

$$\tau_r = \eta_g\,K_g\,\tau_{ml}$$

**Interconnection law**

$$\omega_m=K_g\,\omega$$

**Mass center position of the rod**

$$x_p=L_r \theta-L_c \sin\alpha$$

$$y_p=L_c\cos\alpha$$

**Rod Dynamics**

$$m_p\ddot x_p=F_h$$

$$m_p\ddot y_p=F_v-m_p g$$

$$J_p\ddot\alpha+B_p\dot\alpha=F_v L_c \sin\alpha+F_h L_c \cos\alpha$$

**Rotary Arm Dynamics**

$$J_r\ddot\theta+B_r\dot\theta+A_c\mathbb{sign}(\omega)+F_hL_r=\tau_r$$



### Modeling of Pendulum

**Electrical circuit**

$$L_m\frac{dI_m}{dt} + R_m I_m + k_m \omega_m = V_m$$

Ignore the inductance of motor: $$R_m I_m + k_m \omega_m = V_m$$

$$\tau_m = \eta_m k_t\, I_m$$

**Motor shaft**

$$J_m\dot\omega_m + B_m \omega_m + \tau_{ml} = \tau_m$$

Ignore the moment of inertia and the viscous damping coefficient of motor: $$\tau_{ml} = \tau_m$$

$$\tau_r = \eta_g\,K_g\,\tau_{ml}$$

**Interconnection law**

$$\omega_m=K_g\,\omega$$

**Mass center position of the rod**

$$x_p=L_r \theta+L_c \sin\alpha_1$$

$$y_p=-L_c\cos\alpha_1$$

**Rod Dynamics**

$$m_p\ddot x_p=F_h$$

$$m_p\ddot y_p=F_v-m_p g$$

$$J_p\ddot\alpha_1+B_p\dot\alpha_1=-F_v L_c \sin\alpha_1-F_h L_c \cos\alpha_1$$

**Rotary Arm Dynamics**

$$J_r\ddot\theta+B_r\dot\theta+A_c \mathbb{sign}(\omega)+F_hL_r=\tau_r$$





## B. Dynamics 

### nonlinear dynamics of the inverted pendulum system

$$(J_r+m_pL_r^2)\ddot\theta-m_pL_rL_c\cos\alpha\,\ddot\alpha+(B_r+\dfrac{k_mk_t\eta_m\eta_gK_g^2}{R_m})\dot\theta+m_pL_rL_c\sin\alpha\,\dot\alpha^2+A_c \mathbb{sign}(\omega)=\dfrac{k_t\eta_m\eta_gK_g}{R_m}V_m$$

$$-m_pL_rL_c\cos\alpha\,\ddot\theta+(J_p+m_pL_c^2)\ddot\alpha+B_p\dot\alpha-m_pgL_c\sin\alpha=0$$

### linearized dynamics of the inverted pendulum system

$$(J_r+m_pL_r^2)\ddot\theta-m_pL_rL_c\ddot\alpha+(B_r+\dfrac{k_mk_t\eta_m\eta_gK_g^2}{R_m})\dot\theta=\dfrac{k_t\eta_m\eta_gK_g}{R_m}V_m$$

$$-m_pL_rL_c\ddot\theta+(J_p+m_pL_c^2)\ddot\alpha+B_p\dot\alpha-m_pgL_c\alpha=0$$



### nonlinear dynamics of the pendulum system

$$(J_r+m_pL_r^2)\ddot\theta+m_pL_rL_c\cos\alpha_1\,\ddot\alpha_1+(B_r+\dfrac{k_mk_t\eta_m\eta_gK_g^2}{R_m})\dot\theta-m_pL_rL_c\sin\alpha_1\,\dot\alpha_1^2+A_c \mathbb{sign}(\omega)=\dfrac{k_t\eta_m\eta_gK_g}{R_m}V_m$$

$$m_pL_rL_c\cos\alpha_1\,\ddot\theta+(J_p+m_pL_c^2)\ddot\alpha_1+B_p\dot\alpha_1+m_pgL_c\sin\alpha_1=0$$

### linearized dynamics of the pendulum system

$$(J_r+m_pL_r^2)\ddot\theta+m_pL_rL_c\ddot\alpha_1+(B_r+\dfrac{k_mk_t\eta_m\eta_gK_g^2}{R_m})\dot\theta=\dfrac{k_t\eta_m\eta_gK_g}{R_m}V_m$$

$$m_pL_rL_c\ddot\theta+(J_p+m_pL_c^2)\ddot\alpha_1+B_p\dot\alpha_1+m_pgL_c\alpha_1=0$$





## C. MATLAB/Simulink model for simulation

```matlab
%sfun_Inverted_Pendulumn_Dynamics.m
%PhyPara=[mass,bv,Af,Dc,dr,L0,Betae,Kq1,Kq2,taov,kv]
%InitX=[theta0,theta0_dot,alpha0,alpha0_dot] %deg
%传递参数:倒立摆系统的模型参数(结构体表示)TFPara
function [sys,x0,str,ts,simStateCompliance] = sfun_Inverted_Pendulumn_Dynamics(t,x,u,flag,InitX,TFPara,ModelSelFlag)

switch flag

  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(InitX);

  case 1
    sys=mdlDerivatives(t,x,u,TFPara,ModelSelFlag);

  case 2
    sys=mdlUpdate(t,x,u);

  case 3
    sys=mdlOutputs(t,x,u);

  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(InitX)

sizes = simsizes;

sizes.NumContStates  = 4; %连续状态变量个数
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4; %系统输出变量个数
sizes.NumInputs      = 2; %系统输入，即激励电压和干扰 [uin,dis]
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);
DegToRadCoef=1/180*pi;
theta0 = InitX(1)*DegToRadCoef;
theta0_dot = InitX(2)*DegToRadCoef;
alpha0 = InitX(3)*DegToRadCoef;
alpha0_dot = InitX(4)*DegToRadCoef;
%状态变量初始值设置
x0  = [theta0,theta0_dot,alpha0,alpha0_dot]';
str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u,TFPara,ModelSelFlag)
 % 状态变量
 theta     = x(1);   % 旋臂角
 theta_dot = x(2);   % 旋臂角速度
 alpha     = x(3);   % 摆杆角
 alpha_dot = x(4);   % 摆杆角速度

 % 输入
 Vin = u(1);   % 电机输入电压
 dis = u(2);   % 外部扰动（可选，看模型需要）

 % 提取参数
 Jr   = TFPara.Jr;
 Jp   = TFPara.Jp;
 mp   = TFPara.mp;
 Lr   = TFPara.Lr;
 Lc   = TFPara.Lc;
 Br   = TFPara.Br;
 Bp   = TFPara.Bp;
 km   = TFPara.km;
 kt   = TFPara.kt;
 Rm   = TFPara.Rm;
 Kg   = TFPara.Kg;
 eta_m= TFPara.eta_m;
 eta_g= TFPara.eta_g;
 g    = TFPara.g;
 Ac   = TFPara.Ac;   % 底座摩擦项

 % 电机输入项
 tau_in = (kt*eta_m*eta_g*Kg/Rm) * Vin ...
          - (km*kt*eta_m*eta_g*Kg^2/Rm) * theta_dot;

 % 定义中间系数
 M11 = Jr + mp*Lr^2;
 M12 = -mp*Lr*Lc*cos(alpha);
 M21 = -mp*Lr*Lc*cos(alpha);
 M22 = Jp + mp*Lc^2;

 % 非线性或线性化模型选择
 if(ModelSelFlag == 1) % 非线性倒立摆模型
     f1 = tau_in - Br*theta_dot - mp*Lr*Lc*sin(alpha)*alpha_dot^2 ...
          - Ac*sign(theta_dot);
     f2 = -Bp*alpha_dot + mp*g*Lc*sin(alpha);
     % 解二阶微分方程组
     A = [M11 M12; M21 M22];
     b = [f1; f2];
     dd = A\b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);

 elseif(ModelSelFlag == 2) % 线性化倒立摆模型
     f1 = tau_in - Br*theta_dot;
     f2 = -Bp*alpha_dot + mp*g*Lc*alpha;
     A = [M11 -mp*Lr*Lc; -mp*Lr*Lc M22];
     b = [f1; f2];
     dd = A\b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);

 elseif(ModelSelFlag == 3) % 非线性单摆模型
     f1 = tau_in - Br*theta_dot + mp*Lr*Lc*sin(alpha)*alpha_dot^2 ...
          - Ac*sign(theta_dot);
     f2 = -Bp*alpha_dot - mp*g*Lc*sin(alpha);
     A = [M11 -M12; -M21 M22];
     b = [f1; f2];
     dd = A\b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);

 elseif(ModelSelFlag == 4) % 线性化单摆模型
     f1 = tau_in - Br*theta_dot;
     f2 = -Bp*alpha_dot - mp*g*Lc*alpha;
     A = [M11 mp*Lr*Lc; mp*Lr*Lc M22];
     b = [f1; f2];
     dd = A\b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);

 else
     theta_ddot = 0;
     alpha_ddot = 0;
 end

 % 输出状态导数
 sys(1,1) = theta_dot;
 sys(2,1) = theta_ddot;
 sys(3,1) = alpha_dot;
 sys(4,1) = alpha_ddot;



function sys=mdlUpdate(t,x,u)

sys = [];

function sys=mdlOutputs(t,x,u)

RadToDegCoef = 1/pi*180;
sys = [x(1) x(2) x(3) x(4)]*RadToDegCoef;


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;   
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];
```



