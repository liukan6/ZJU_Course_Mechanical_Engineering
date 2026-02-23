%PhyPara=[mass,bv,Af,Dc,dr,L0,Betae,Kq1,Kq2,taov,kv]
%InitX=[alpha0,alpha0_dot,theta0,theta0_dot] %deg
function [sys,x0,str,ts,simStateCompliance] = sfun_Inverted_Pendulumn_Dynamics(t,x,u,flag,InitX,TFPara,ModelSelFlag)

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

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 2; %[uin,dis]
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);
DegToRadCoef=1/180*pi;
theta0 = InitX(1)*DegToRadCoef;
theta0_dot = InitX(2)*DegToRadCoef;
alpha0 = InitX(3)*DegToRadCoef;
alpha0_dot = InitX(4)*DegToRadCoef;
x0  = [theta0,theta0_dot,alpha0,alpha0_dot]';
str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u,TFPara,ModelSelFlag)
     N1 = TFPara.Taot/TFPara.Kt;
     N2 = 1/TFPara.Kt;
     N3 = TFPara.Ac;%/TFPara.Ku;  
     N5 = TFPara.deta_theta;
    if (ModelSelFlag >0)
      N4 = TFPara.Ka;   
      N6 = TFPara.Kr*TFPara.wr^2;
      N7 = 2*TFPara.kesair*TFPara.wr;
     N8 = TFPara.wr^2;
     N9 = TFPara.deta_alpha;
    end
     uinDis = u(2);
     Voltage = u(1);
     theta = x(1);
     theta_dot = x(2);
     alpha = x(3);
     alpha_dot =x(4);
     signcoef = 1e3;
     % MM4 = [0
     %        -uinDis];
      MM4 = [-uinDis
            0];
     Fcolum = tanh(theta_dot*signcoef);
  if(ModelSelFlag == 0) %Sero table without rod
      sys(1,1) = theta_dot;
      sys(2,1) = 1/N1*(Voltage - N2*theta_dot - N3*Fcolum - N5);
      sys(3,1) = 0;
      sys(4,1) = 0;
    %  sys(4,1) = (Ku*Voltage-uinDis-Ku*AcFric*tanh(theta_dot*signcoef)-Bt*theta_dot)/Jt;
      return;
  elseif(ModelSelFlag == 1) %nonlinear Inverted Pendulum 
%      MM1 = [Ip        -A3*cos(alpha)
%           -A3*cos(alpha)     It        ];
%      MM2 = [-Bp*alpha_dot + A2*sin(alpha)
%             -Bt*theta_dot - A3*sin(alpha)*alpha_dot^2];   
     MM1 = [N1 -N4*cos(alpha)
            N6*cos(alpha) -1];
     MM2= [-N3*Fcolum-N2*theta_dot-N4*sin(alpha)*alpha_dot^2+Voltage-N5
           -N8*sin(alpha)+N7*alpha_dot-N9];

  elseif(ModelSelFlag == 2) %linear Inverted Pendulum 
%      MM1 = [Ip     -A3
%            -A3      It   ];
%      MM2 = [-Bp*alpha_dot + A2*alpha
%             -Bt*theta_dot];
    MM1 = [N1 -N4
           N6 -1];
    MM2 = [-N3*Fcolum-N2*theta_dot+Voltage-N5
          -N8*alpha+N7*alpha_dot-N9];

  elseif(ModelSelFlag == 3) %nonlinear pendulum 
%      MM1 = [Ip             A3*cos(alpha)
%            A3*cos(alpha)     It        ];
%      MM2 = [-Bp*alpha_dot - A2*sin(alpha)
%             -Bt*theta_dot - A3*sin(alpha)*alpha_dot^2]; 
     MM1 = [N1 N4*cos(alpha)
           -N6*cos(alpha) -1];
     MM2 = [-N3*Fcolum-N2*theta_dot-N4*sin(alpha)*alpha_dot^2+Voltage-N5
           N8*sin(alpha)+N7*alpha_dot-N9];

  elseif(ModelSelFlag == 4) %linear pendulum
%      MM1 = [Ip     A3
%             A3     It   ];
%      MM2 = [-Bp*alpha_dot - A2*alpha
%             -Bt*theta_dot]; 
     MM1 = [N1 N4
           -N6 -1];
     MM2 = [-N3*Fcolum-N2*theta_dot+Voltage-N5
           N8*alpha+N7*alpha_dot-N9];
  else
      MM1=diag([1,1]');
      MM2 = [0,0]';
      MM4 = [0,0]';
  end
  yy = inv(MM1)*(MM2 + MM4);

 sys(1,1) = theta_dot;
 sys(2,1) = yy(1);
 sys(3,1) = alpha_dot;
 sys(4,1) = yy(2);


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
