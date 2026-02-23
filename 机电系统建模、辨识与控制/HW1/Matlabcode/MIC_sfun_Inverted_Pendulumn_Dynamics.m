%PhyPara=[mass,bv,Af,Dc,dr,L0,Betae,Kq1,Kq2,taov,kv]
%InitX=[theta0,theta0_dot,alpha0,alpha0_dot] %deg
%传递参数:倒立摆系统的模型参数(结构体表示)TFPara
function [sys,x0,str,ts,simStateCompliance] = MIC_sfun_Inverted_Pendulumn_Dynamics(t,x,u,flag,InitX,TFPara,ModelSelFlag)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(InitX);

  case 1,
    sys=mdlDerivatives(t,x,u,TFPara,ModelSelFlag);

  case 2,
    sys=mdlUpdate(t,x,u);


  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sys=mdlDerivatives(t,x,u,TFPara,ModelSelFlag)
 %To Do write down the derivative functions of theta and alpha for different system

 %To Do 
 %根据倒立摆/单摆系统数学模型，在下面填写与TFPara相关的theta_dot,theta_ddot,alpha_dot,alpha_ddot数学表达式

 % 提取状态变量
 theta = x(1);      % 旋臂角度 (rad)
 theta_dot = x(2);  % 旋臂角速度 (rad/s)
 alpha = x(3);      % 摆杆角度 (rad)
 alpha_dot = x(4);  % 摆杆角速度 (rad/s)
 
 % 提取输入信号
 Vin = u(1);        % 控制电压 (V)
 dis = u(2);        % 干扰信号
 
 % 提取系统参数（根据mlx文件中的参数命名）
 mp = TFPara.mp;    % 摆杆质量
 Lc = TFPara.Lc;    % 摆杆质心到转轴距离
 Jp = TFPara.Jp;    % 摆杆转动惯量
 Bp = TFPara.Bp;    % 摆杆阻尼系数
 Lt = TFPara.Lt;    % 旋臂长度
 Jeq = TFPara.Jeq;  % 旋臂等效转动惯量
 Beq = TFPara.Beq;  % 旋臂阻尼系数
 Rm = TFPara.Rm;    % 电机电阻
 kt = TFPara.kt;    % 电机转矩常数
 km = TFPara.km;    % 电机反电动势常数
 Kg = TFPara.Kg;    % 齿轮减速比
 yitam = TFPara.yitam; % 电机效率
 yitag = TFPara.yitag; % 齿轮效率
 g = TFPara.Accg;   % 重力加速度
 Ac = TFPara.Ac;    % 库仑摩擦系数
 
 % 计算电机转矩（参考模型中的公式）
 tau_in = (kt*yitam*yitag*Kg/Rm) * Vin - (km*kt*yitam*yitag*Kg^2/Rm) * theta_dot + dis;
 
 % 定义质量矩阵元素（使用mlx文件中的符号）
 Jr = Jeq;  % 旋臂转动惯量
 Lr = Lt;   % 旋臂长度
 
 M11 = Jr + mp*Lr^2;
 M12 = -mp*Lr*Lc*cos(alpha);
 M21 = -mp*Lr*Lc*cos(alpha);
 M22 = Jp + mp*Lc^2;
 
 % 根据模型选择标志计算动力学方程
 if(ModelSelFlag == 1) %非线性倒立摆模型
     % 力向量（非线性项完整）
     f1 = tau_in - Beq*theta_dot - mp*Lr*Lc*sin(alpha)*alpha_dot^2 - Ac*sign(theta_dot);
     f2 = -Bp*alpha_dot + mp*g*Lc*sin(alpha);
     
     % 构建质量矩阵和力向量
     A = [M11, M12;
          M21, M22];
     b = [f1; f2];
     
     % 求解加速度
     dd = A \ b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);
     
 elseif(ModelSelFlag == 2) %线性化倒立摆模型
     % 线性化力向量（小角度假设：sin(alpha)≈alpha, cos(alpha)≈1）
     f1 = tau_in - Beq*theta_dot - Ac;
     f2 = -Bp*alpha_dot + mp*g*Lc*alpha;
     
     % 线性化质量矩阵
     A = [M11, -mp*Lr*Lc;  % cos(alpha)≈1
          -mp*Lr*Lc, M22]; % cos(alpha)≈1
     b = [f1; f2];
     
     % 求解加速度
     dd = A \ b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);
     
 elseif(ModelSelFlag == 3) %非线性单摆模型（旋臂固定）
     % 力向量（非线性项完整）
     f1 = tau_in - Beq*theta_dot + mp*Lr*Lc*sin(alpha)*alpha_dot^2 - Ac*sign(theta_dot);
     f2 = -Bp*alpha_dot - mp*g*Lc*sin(alpha);
     
     % 构建质量矩阵和力向量
     A = [M11, -M12;
          -M21, M22];
     b = [f1; f2];
     
     % 求解加速度
     dd = A \ b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);
     
 elseif(ModelSelFlag == 4) %线性化单摆模型
     % 线性化力向量（小角度假设：sin(alpha)≈alpha, cos(alpha)≈1）
     f1 = tau_in - Beq*theta_dot - Ac;
     f2 = -Bp*alpha_dot - mp*g*Lc*alpha;
     
     % 线性化质量矩阵
     A = [M11, mp*Lr*Lc;  % cos(alpha)≈1
          mp*Lr*Lc, M22]; % cos(alpha)≈1
     b = [f1; f2];
     
     % 求解加速度
     dd = A \ b;
     theta_ddot = dd(1);
     alpha_ddot = dd(2);
     
 else
     % 默认情况，报错
     error('Invalid ModelSelFlag. Use 1-4 for different models.');
 end


 sys(1,1) = theta_dot;  %旋臂角速度
 sys(2,1) = theta_ddot; %旋臂角加速度
 sys(3,1) = alpha_dot;  %摆杆角速度
 sys(4,1) = alpha_ddot; %摆杆角加速度


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
