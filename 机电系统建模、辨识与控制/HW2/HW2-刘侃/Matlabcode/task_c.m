%% run_identification_cases.m
% 运行 7 个 case，保存每个 case 的验证图，输出参数对比表
% 直接运行前请确保工作区已有 NominalPara, IdentSet, UinData, InitX, DisturbPara, SimuRunInfo, ModelSelFlag

% ---- 1) 检查工作区必需变量 ----
requiredVars = {'NominalPara','IdentSet','UinData','InitX','DisturbPara','SimuRunInfo','ModelSelFlag'};
for k=1:numel(requiredVars)
    if ~exist(requiredVars{k},'var')
        error('Missing required workspace variable: %s. 请先在 workspace 中准备好该变量并重试。', requiredVars{k});
    end
end

% ---- 2) 定义 7 个 case ----
cases = struct('name',[],'sigma',[],'wf',[],'zeta',[],'Ad',[],'fd',[]);
cases(1) = struct('name','Case1','sigma',0,'wf',15,'zeta',0.7,'Ad',0,'fd',1);
cases(2) = struct('name','Case2','sigma',0,'wf',5 ,'zeta',0.7,'Ad',0,'fd',1);
cases(3) = struct('name','Case3','sigma',0,'wf',300,'zeta',0.7,'Ad',0,'fd',1);
cases(4) = struct('name','Case4','sigma',1,'wf',15,'zeta',0.7,'Ad',0,'fd',1);
cases(5) = struct('name','Case5','sigma',3,'wf',15,'zeta',0.7,'Ad',0,'fd',1);
cases(6) = struct('name','Case6','sigma',0,'wf',15,'zeta',0.7,'Ad',0.1,'fd',1);
cases(7) = struct('name','Case7','sigma',0,'wf',15,'zeta',0.7,'Ad',0.5,'fd',1);

% 预分配结果存储
ResultTable = table('Size',[numel(cases),8],...
    'VariableTypes',repmat("double",1,8),...
    'VariableNames',{'tau_t','Kbar_t','Ac','Kbar_a','wr','zeta_r','L_over_g','case_index'});
ResultTable.case_index = (1:numel(cases))';

% ---- 3) 主循环：对每个 case 执行 ----
for iCase = 1:numel(cases)
    C = cases(iCase);
    fprintf('\n=== Running %s: sigma=%.3g, wf=%.1f, zeta=%.2f, Ad=%.3g ===\n', C.name, C.sigma, C.wf, C.zeta, C.Ad);
    
    % 3.1 复制原始设置以免污染
    NominalPara_i = NominalPara;
    IdentSet_i = IdentSet;
    DisturbPara_i = DisturbPara;
    UinData_i = UinData;
    
    % 3.2 尝试把滤波器参数写入 IdentSet（做多种兼容写法）
    if isfield(IdentSet_i,'omega_f'), IdentSet_i.omega_f = C.wf; end
    if isfield(IdentSet_i,'wf'), IdentSet_i.wf = C.wf; end
    if isfield(IdentSet_i,'wc'), IdentSet_i.wc = C.wf; end
    % 也放一些可能的命名
    IdentSet_i.filter_omega = C.wf;
    IdentSet_i.filter_zeta = C.zeta;
    IdentSet_i.zeta_f = C.zeta;
    
    % 3.3 运行一次 nominal 仿真（得到原始信号 IPResData）
    IPResData = Ident_SimulateRun(NominalPara_i, ModelSelFlag, UinData_i, InitX, DisturbPara_i, SimuRunInfo);
    
    % 确保 time 字段存在
    if ~isfield(IPResData,'time')
        error('IPResData 中未包含 time 字段，无法继续。请检查 Ident_SimulateRun 输出。');
    end
    t_ip = IPResData.time(:);
    
    % 3.4 在仿真数据上加入输入扰动（若 Ad > 0）
    if C.Ad ~= 0
        disp('  -> Adding input disturbance to uin (Ad*sin(2*pi*fd*t))');
        dt_signal = C.Ad * sin(2*pi*C.fd*t_ip);
        if isfield(IPResData,'uin')
            IPResData.uin = IPResData.uin(:) + dt_signal;
        elseif isfield(UinData_i,'uin') && isfield(IPResData,'time') % 有时 uin 存在于 UinData
            % create/override
            IPResData.uin = dt_signal;
        else
            % create uin field
            IPResData.uin = dt_signal;
        end
    end
    
    % 3.5 在测量信号上加入测量噪声（若 sigma > 0）
    sigma = C.sigma;
    if sigma ~= 0
        disp('  -> Adding measurement noise to available signals (theta, alpha, derivatives, uin)');
        % 列出可能的字段名并加噪声（若存在）
        candidateFields = {'uin','theta','thetadot','thetaddot','alpha','alphadot','alphaddot'};
        for ff = candidateFields
            f = ff{1};
            if isfield(IPResData,f)
                v = IPResData.(f);
                if ~isempty(v)
                    IPResData.(f) = v + sigma*randn(size(v));
                end
            end
        end
    end
    
    % 3.6 通过滤波（Ident_FilterRun）得到 filtered 数据
    % 注意：Ident_FilterRun 可能依赖 IdentSet 中的滤波参数
    try
        FilterResData = Ident_FilterRun(IPResData, IdentSet_i, SimuRunInfo, ModelSelFlag);
    catch ME
        warning('Ident_FilterRun 失败：%s\n尝试直接用 IPResData 作为 filtered 数据的近似。', ME.message);
        % 如果失败，try to map raw names to *_f names
        FilterResData = IPResData;
        % create *_f copies if missing
        if isfield(IPResData,'uin') && ~isfield(FilterResData,'uin_f'), FilterResData.uin_f = IPResData.uin; end
        if isfield(IPResData,'theta') && ~isfield(FilterResData,'theta'), FilterResData.theta = IPResData.theta; end
        if isfield(IPResData,'alpha') && ~isfield(FilterResData,'alpha'), FilterResData.alpha = IPResData.alpha; end
        % derivatives
        if isfield(IPResData,'thetadot') && ~isfield(FilterResData,'thetadot_f'), FilterResData.thetadot_f = IPResData.thetadot; end
        if isfield(IPResData,'thetaddot') && ~isfield(FilterResData,'thetaddot_f'), FilterResData.thetaddot_f = IPResData.thetaddot; end
        if isfield(IPResData,'alphadot') && ~isfield(FilterResData,'alphadot_f'), FilterResData.alphadot_f = IPResData.alphadot; end
        if isfield(IPResData,'alphaddot') && ~isfield(FilterResData,'alphaddot_f'), FilterResData.alphaddot_f = IPResData.alphaddot; end
        if isfield(IPResData,'alpha') && ~isfield(FilterResData,'alpha_f'), FilterResData.alpha_f = IPResData.alpha; end
    end
    
    % 3.7 将之前你提供的“回归并映射回物理量”的代码封装成局部函数（inline）
    [IdentifyPara,IPIdentVerifyData,infoStruct] = local_identify_from_filtered(FilterResData, NominalPara_i, ModelSelFlag, UinData_i, InitX, DisturbPara_i, SimuRunInfo);
    
    % 3.8 保存结果到表格
    ResultTable.tau_t(iCase) = getfield_safe(IdentifyPara,'Taot', NaN);
    ResultTable.Kbar_t(iCase) = getfield_safe(IdentifyPara,'Kt', NaN);
    ResultTable.Ac(iCase) = getfield_safe(IdentifyPara,'Ac', NaN);
    ResultTable.Kbar_a(iCase) = getfield_safe(IdentifyPara,'Ka', NaN);
    ResultTable.wr(iCase) = getfield_safe(IdentifyPara,'wr', NaN);
    ResultTable.zeta_r(iCase) = getfield_safe(IdentifyPara,'kesair', NaN);
    ResultTable.L_over_g(iCase) = getfield_safe(IdentifyPara,'Kr', NaN);
    
    % 3.9 绘图与保存（使用之前增强绘图布局）
    % Build a figure similar to earlier enhanced plotting but with title including case
    figure('Name',sprintf('%s Validation',C.name),'NumberTitle','off','Units','normalized','Position',[0.05 0.05 0.9 0.85]);
    t_meas = FilterResData.time(:);
    % get filtered signals with multiple name attempts
    uin_f = getfield_safe(FilterResData,'uin_f', getfield_safe(FilterResData,'uin', NaN(size(t_meas))));
    theta_meas = getfield_safe(FilterResData,'theta', getfield_safe(FilterResData,'theta_f', NaN(size(t_meas))));
    alpha_meas = getfield_safe(FilterResData,'alpha', getfield_safe(FilterResData,'alpha_f', NaN(size(t_meas))));
    
    theta_sim = NaN(size(t_meas));
    alpha_sim = NaN(size(t_meas));
    if isfield(IPIdentVerifyData,'time') && isfield(IPIdentVerifyData,'theta')
        theta_sim = interp1(IPIdentVerifyData.time(:), IPIdentVerifyData.theta(:), t_meas, 'linear', 'extrap');
    end
    if isfield(IPIdentVerifyData,'time') && isfield(IPIdentVerifyData,'alpha')
        alpha_sim = interp1(IPIdentVerifyData.time(:), IPIdentVerifyData.alpha(:), t_meas, 'linear', 'extrap');
    end
    
    % Residuals
    res_theta = theta_meas - theta_sim;
    res_alpha = alpha_meas - alpha_sim;
    
    % Subplots (3x2)
    subplot(3,2,1);
    plot(t_meas, uin_f, 'LineWidth',1.2); xlabel('Time (s)'); ylabel('u (filtered)'); title([C.name ' - excitation (filtered)']);
    subplot(3,2,2);
    hold on;
    plot(t_meas, theta_meas, '-', 'LineWidth',1.2);
    if ~all(isnan(theta_sim)), plot(t_meas, theta_sim, '--','LineWidth',1); end
    hold off; xlabel('Time (s)'); ylabel('\theta'); title('Table angle: measured (filtered) vs sim'); legend({'meas (f)','sim'});
    subplot(3,2,3);
    hold on;
    plot(t_meas, alpha_meas, '-', 'LineWidth',1.2);
    if ~all(isnan(alpha_sim)), plot(t_meas, alpha_sim, '--','LineWidth',1); end
    hold off; xlabel('Time (s)'); ylabel('\alpha'); title('Rod angle: measured (filtered) vs sim'); legend({'meas (f)','sim'});
    subplot(3,2,4);
    plot(t_meas, res_theta, 'LineWidth',1); xlabel('Time (s)'); ylabel('res \theta'); title('Residual: theta');
    subplot(3,2,5);
    plot(t_meas, res_alpha, 'LineWidth',1); xlabel('Time (s)'); ylabel('res \alpha'); title('Residual: alpha');
    subplot(3,2,6);
    hold on;
    if any(~isnan(res_theta)), histogram(res_theta(~isnan(res_theta)),'Normalization','pdf'); end
    if any(~isnan(res_alpha)), histogram(res_alpha(~isnan(res_alpha)),'Normalization','pdf'); end
    hold off; title('Residual histograms'); legend({'res\_theta','res\_alpha'});
    
    sgtitle(sprintf('%s: Ident Results (tau=%.4g, Kt=%.4g, Ac=%.4g, wr=%.4g)', C.name, ResultTable.tau_t(iCase), ResultTable.Kbar_t(iCase), ResultTable.Ac(iCase), ResultTable.wr(iCase)));
    
    % 保存图片
    outname = sprintf('%s_validation.png', C.name);
    try
        exportgraphics(gcf, outname, 'Resolution',300);
    catch
        saveas(gcf,outname);
    end
    fprintf('  -> Saved figure to %s\n', outname);
    
    % pause briefly to ensure figures render (可移除)
    pause(0.2);
end

% ---- 4) 输出结果表格并画参数趋势 ----
disp('=== Identification results summary ===');
disp(ResultTable);

% 绘参数趋势图
figure('Name','Parameter trends','NumberTitle','off','Units','normalized','Position',[0.05 0.05 0.9 0.5]);
subplot(2,4,1); plot(ResultTable.case_index, ResultTable.tau_t,'-o'); xlabel('case'); title('\tau_t'); grid on;
subplot(2,4,2); plot(ResultTable.case_index, ResultTable.Kbar_t,'-o'); xlabel('case'); title('Kbar_t'); grid on;
subplot(2,4,3); plot(ResultTable.case_index, ResultTable.Ac,'-o'); xlabel('case'); title('A_c'); grid on;
subplot(2,4,4); plot(ResultTable.case_index, ResultTable.Kbar_a,'-o'); xlabel('case'); title('Kbar_a'); grid on;
subplot(2,4,5); plot(ResultTable.case_index, ResultTable.wr,'-o'); xlabel('case'); title('w_r'); grid on;
subplot(2,4,6); plot(ResultTable.case_index, ResultTable.zeta_r,'-o'); xlabel('case'); title('\zeta_r'); grid on;
subplot(2,4,7); plot(ResultTable.case_index, ResultTable.L_over_g,'-o'); xlabel('case'); title('L_t/g'); grid on;
sgtitle('Estimated parameters across cases');

try
    exportgraphics(gcf,'params_trends.png','Resolution',300);
    fprintf('Saved aggregated parameter trends to params_trends.png\n');
catch
    saveas(gcf,'params_trends.png');
end

%% --------- 本文件内辅助函数定义（局部函数） ----------
function val = getfield_safe(s, fname, defaultVal)
    if isstruct(s) && isfield(s,fname)
        val = s.(fname);
    else
        val = defaultVal;
    end
end

function [IdentifyPara, IPIdentVerifyData, info] = local_identify_from_filtered(data, NominalPara, ModelSelFlag, UinData, InitX, DisturbPara, SimuRunInfo)
% 基于你提供的回归逻辑，从 FilterResData 得到 IdentifyPara，并运行一次辨识后仿真
    info = struct();
    IdentifyPara = NominalPara; % 默认回退
    
    % 取出必要信号（兼容多种命名）
    t = getfield_safe(data,'time', []);
    if isempty(t), error('Filtered data 没有 time 字段。'); end
    % input
    uin = getfield_safe(data,'uin_f', getfield_safe(data,'uin', NaN(size(t))));
    % theta derivatives / signals
    theta_dd = getfield_safe(data,'thetaddot_f', getfield_safe(data,'thetaddot', NaN(size(t))));
    theta_d  = getfield_safe(data,'thetadot_f', getfield_safe(data,'thetadot', NaN(size(t))));
    theta    = getfield_safe(data,'theta', getfield_safe(data,'theta_f', NaN(size(t))));
    % alpha signals
    alpha_dd = getfield_safe(data,'alphaddot_f', getfield_safe(data,'alphaddot', NaN(size(t))));
    alpha_d  = getfield_safe(data,'alphadot_f', getfield_safe(data,'alphadot', NaN(size(t))));
    alpha    = getfield_safe(data,'alpha_f', getfield_safe(data,'alpha', NaN(size(t))));
    
    % 均截取有效样点
    valid = ~isnan(uin) & ~isnan(theta_dd) & ~isnan(alpha_dd);
    t = t(valid); uin = uin(valid);
    theta_dd = theta_dd(valid); theta_d = theta_d(valid);
    alpha_dd = alpha_dd(valid); alpha_d = alpha_d(valid); alpha = alpha(valid);
    
    % ---- 1) servo table dynamics regression ----
    Phi1 = [ theta_dd, theta_d, sign(theta_d), alpha_dd ]; % N x 4
    y1   = uin;
    beta1 = (Phi1' * Phi1) \ (Phi1' * y1);
    beta_tau_over_Kbar = beta1(1);
    beta_invKbar        = beta1(2);
    beta_Ac             = beta1(3);
    beta_Kabar          = beta1(4);
    if abs(beta_invKbar) < eps
        warning('估计到 1/Kbar_t 非常小或为零，无法可靠反求 Kbar_t。保留 Nominal 值并继续。');
        Kbar_t_est = NominalPara.Kt;
    else
        Kbar_t_est = 1 / beta_invKbar;
    end
    tau_t_est = beta_tau_over_Kbar * Kbar_t_est;
    Ac_est    = beta_Ac;
    Kabar_est = beta_Kabar;
    
    % ---- 2) rod dynamics regression ----
    Phi2 = [ -theta_dd, -alpha_d, -alpha ];
    y2   = alpha_dd;
    beta2 = (Phi2' * Phi2) \ (Phi2' * y2);
    beta_Loverg_wr2 = beta2(1);
    beta_2zwr       = beta2(2);
    beta_wr2        = beta2(3);
    if beta_wr2 < 0
        warning('估计得到 ω_r^2 为负，取绝对值后开方以获得实数 ω_r（请检查数据）。');
        beta_wr2 = abs(beta_wr2);
    end
    wr_est = sqrt(beta_wr2);
    if wr_est ~= 0
        zeta_r_est = beta_2zwr / (2 * wr_est);
    else
        zeta_r_est = NaN;
    end
    if beta_wr2 ~= 0
        Loverg_est = beta_Loverg_wr2 / beta_wr2;
    else
        Loverg_est = NaN;
    end
    
    % ---- 3) 写回 IdentifyPara ----
    IdentifyPara = NominalPara;
    IdentifyPara.Taot = tau_t_est;
    IdentifyPara.Kt   = Kbar_t_est;
    IdentifyPara.Ac   = Ac_est;
    IdentifyPara.Ka   = Kabar_est;
    IdentifyPara.wr   = wr_est;
    IdentifyPara.kesair = zeta_r_est;
    IdentifyPara.Kr = Loverg_est;
    IdentifyPara.wt = 1 / max(eps, IdentifyPara.Taot);
    IdentifyPara.deta_alpha = 0;
    IdentifyPara.deta_theta = 0;
    
    % ---- 4) 用 IdentifyPara 做一次仿真并返回 ----
    try
        IPIdentVerifyData = Ident_SimulateRun(IdentifyPara, ModelSelFlag, UinData, InitX, DisturbPara, SimuRunInfo);
    catch ME
        warning('用辨识参数仿真失败：%s\n将返回空仿真结构。', ME.message);
        IPIdentVerifyData = struct('time',t,'theta',nan(size(t)),'alpha',nan(size(t)));
    end
    
    % 返回 info（保留中间 beta）
    info.beta1 = beta1;
    info.beta2 = beta2;
end


 function [TFPara] = Ident_GetTFParaFromPhyPara(PhyPara)
    mp = PhyPara.mp;
    Lc = PhyPara.Lc;
    Jp = PhyPara.Jp;
    Bp = PhyPara.Bp;
    Lt = PhyPara.Lt;
    Jeq = PhyPara.Jeq;
    Beq = PhyPara.Beq;
    Rm = PhyPara.Rm;
    kt = PhyPara.kt;
    km = PhyPara.km;
    Kg = PhyPara.Kg;
    yitam = PhyPara.yitam;
    yitag = PhyPara.yitag;
    Accg = PhyPara.Accg;
    Ac = PhyPara.AcFric;
    
     Jt = Jeq;
     Ip = Jp + mp*Lc^2; %A1
     It = Jt + mp*Lt^2; %B1
     Ku = yitam*yitag*kt*Kg/Rm; %B2
     Bt = (yitam*yitag*kt*km*Kg^2+Beq*Rm)/Rm; %Beq;
     clear TFara;
     TFPara = PhyPara;
     TFPara.Kt = Ku/Bt;
     TFPara.Taot = It/Bt;
     TFPara.Ac = Ac;
     TFPara.Ka = mp*Lt*Lc/Ku;
     TFPara.wr = sqrt(mp*Accg*Lc/Ip);
     TFPara.kesair = sqrt(Bp^2/(4*Ip*mp*Accg*Lc));
     TFPara.wt = 1/TFPara.Taot;
     TFPara.Kr = Lt/Accg;
     TFPara.deta_alpha = 0;
     TFPara.deta_theta = 0;
 end


function [UinData] = Ident_SetDefaultUin(UinSet,SimuRunInfo)
  clear UinData;
  %输入正弦信号参数
    UinData.Sin =[1,4*pi,0];%[amp,fre,bias].Amp = 1;%0-2-6;
  %输入方波信号参数 
    UinData.Square = [5,15*pi,0]; %[amp,fre,bias]
  %辨识激励信号(正弦叠加信号)的设置 
    IdentTs = SimuRunInfo.IdentTs;
    identN = 2*floor(SimuRunInfo.TotalTime/IdentTs);%10000; %系统辨识的数据个数
    Identtraj = idinput([identN 1 1],'sine',[0,1],[-1,1],[500 50 1]);
    tout = 0:IdentTs:identN*IdentTs-IdentTs;
    UinData.IdentTrajData.time = tout'; %正弦叠加的辨识激励信号
    UinData.IdentTrajData.signals.values = Identtraj;
    UinData.IdentTrajData.dimensions = 1;
   %选择波形类型  
    UinData.WaveFlag = UinSet.WaveFlag;
  switch (UinSet.WaveFlag)
      case {1} %square
          UinData.Square = [UinSet.Amp,UinSet.Fre,0];%[amp,fre,bias]
      case {2} % uinsin
          UinData.Sin = [UinSet.Amp,UinSet.Fre,0];
      case {3}  %sumsinusoidal
         SumSinFreRange = UinSet.SumSinFreRange/(0.5/IdentTs);
         SumSinAmpRange = [-UinSet.Amp,UinSet.Amp]; %辨识激励信号的范围;
         sumnumber = UinSet.SumSinFreRange(2)*7.5;
         Identtraj = idinput([identN 1 1],'sine',SumSinFreRange,SumSinAmpRange,[sumnumber 50 1]);
         tout = 0:IdentTs:identN*IdentTs-IdentTs;
         UinData.IdentTrajData.time = tout'; %正弦叠加的辨识激励信号
         UinData.IdentTrajData.signals.values = Identtraj;
         UinData.IdentTrajData.dimensions = 1;
      case {4} %custom define uin
        UinData.IdentTrajData.time = UinSet.time;%正弦叠加的辨识激励信号
        UinData.IdentTrajData.signals.values = UinSet.uin;
        UinData.IdentTrajData.dimensions = 1;
        UinData.WaveFlag = 3;
  end
end

function [SysResData]=Ident_SimulateRun(TFPara,ModelSelFlag,UinData,InitX,DisturbPara,SimuRunInfo)
 
  switch ModelSelFlag 
      case 0 %servo table without rod
       ModelSelFlagNonlin = ModelSelFlag;
       ModelSelFlagLin = ModelSelFlag;
      case {1,2} %Inverted Pendulum
       ModelSelFlagNonlin = ModelSelFlag;
       ModelSelFlagLin = 2;   
     case {3,4} %Pendulum
       ModelSelFlagNonlin = ModelSelFlag;
       ModelSelFlagLin = 4;   
 end
  assignin('base', 'WaveFlag',UinData.WaveFlag); 
  assignin('base', 'TFPara',TFPara); 
  assignin('base', 'ModelSelFlagNonlin',ModelSelFlagNonlin);
  assignin('base', 'ModelSelFlagLin',ModelSelFlagLin);
  assignin('base', 'InitX',InitX);  
  assignin('base', 'IdentTs',SimuRunInfo.IdentTs);  
  assignin('base', 'UinSquare',UinData.Square);  
  assignin('base', 'UinSin',UinData.Sin);  
  assignin('base', 'IdentTrajData',UinData.IdentTrajData);  
  assignin('base', 'UinDis',DisturbPara.Dis); 
  assignin('base', 'NoisePara',DisturbPara.Noise); 
  sim('MIC_Mdl_Inverted_Pendulum.slx',SimuRunInfo.TotalTime); %simulink 仿真
    y1 = ResponseData.signals.values;
    t1 = ResponseData.time; 
    clear SysResData;
    %记录响应数据
    SysResData.time = t1;
    SysResData.uin = y1(:,1);
    SysResData.theta = y1(:,2);
    SysResData.thetadot = y1(:,3);
    SysResData.alpha = y1(:,4);
    SysResData.alphadot = y1(:,5);
   
    SysResData.theta_Lin = y1(:,6);
    SysResData.thetadot_Lin = y1(:,7);
    SysResData.alpha_Lin = y1(:,8);
    SysResData.alphadot_Lin = y1(:,9);
end

function [FilterResData]=Ident_FilterRun(ResData,IdentSet,SimuRunInfo,ModelSelFlag)
 if(ModelSelFlag ==1)
    Alpha1Flag = 1; 
 else
    Alpha1Flag = 0;
 end
  clear OriResData;
  OriResData.time = ResData.time;
  OriResData.signals.values = [ResData.uin ResData.alpha ResData.theta ResData.alphadot  ResData.thetadot];%[u2,alpha2,theta2];
  OriResData.signals.dimensions = 5;
  assignin('base', 'OriResData',OriResData); 
  assignin('base', 'Difwn',IdentSet.Difwn); 
  assignin('base', 'DifZeta',IdentSet.DifZeta);  
  assignin('base', 'Alpha1Flag',Alpha1Flag); 
  assignin('base', 'Identwn',IdentSet.Filterwn); 
  assignin('base', 'IdentZeta',IdentSet.FilterZeta);
  assignin('base', 'IdentTs',SimuRunInfo.IdentTs);  

  sim('MIC_Mdl_IdentFilter.slx',SimuRunInfo.TotalTime); %simulink 仿真
    y1 = IdentFilterData.signals.values;
    t1 = IdentFilterData.time; 
    clear FilterResData;
    FilterResData.time = t1;
    FilterResData.uin_f = y1(:,1);
    FilterResData.alpha_f = y1(:,2);
    FilterResData.alphadot_f = y1(:,3);
    FilterResData.alphaddot_f = y1(:,4);
    FilterResData.signalphadot_f = y1(:,5);
    FilterResData.one_f = y1(:,6);
    FilterResData.theta_f = y1(:,7);
    FilterResData.thetadot_f = y1(:,8);
    FilterResData.thetaddot_f = y1(:,9);
    FilterResData.signthetadot_f = y1(:,10);
    FilterResData.uin = y1(:,11);
    FilterResData.alpha = y1(:,12);
    FilterResData.theta = y1(:,13);
    FilterResData.alphadot = y1(:,14);
    FilterResData.thetadot = y1(:,15);
end
