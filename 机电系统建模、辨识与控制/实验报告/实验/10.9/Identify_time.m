%% study_segment_selection.m
clear; clc; close all;

%% 基本参数（按需修改）
taos = 0.005;
zeta = 0.9;
Wf   = 10;      % 固定滤波截止频率（保持滤波一致）
dt   = 0.005;
time = 6;       % 实测数据包含 6 s 数据

% 初始默认（示例）：我们要扫描的 tdown 与 tup 列表
% 说明：确保所有 tup <= time 且 tup > tdown
tdown_list = 0:0.5:4;    % 截取起始时间列表 (s)，可按需修改
tup_list   = 2:0.5:5;    % 截取结束时间列表 (s)，可按需修改

%% 读取实测数据（与你原始代码一致）
data = load('实测数据.txt');
angle_data = (data(1:time/dt+1,1) + 180)/180*pi;   % rad
pos_data   = data(1:time/dt+1,2)/1000;             % m
MotorAcc_data = data(1:time/dt+1,5);

len = size(angle_data,1);
t = 0:dt:dt*(len-1);

% 时间序列信号（供 Simulink 读取）
MotorAcc = [t', MotorAcc_data];
angle    = [t', angle_data];
pos      = [t', pos_data];

% 把工作区变量准备好（Simulink 从 current workspace 读取）
% (这里不使用 assignin，变量本身就在当前 workspace 中)

%% ---------------- 先运行一次滤波器，保证滤波一致 ----------------
fprintf('运行 Filter.slx（一次）以取得滤波结果 ...\n');
outFilter = sim('Filter.slx', 'SrcWorkspace','current');   % 产生 outFilter.Yt, outFilter.x1/x2/x3

% 检查滤波输出长度是否与数据长度一致
if length(outFilter.Yt) < len
    error('Filter 输出长度小于数据长度，请检查 Filter.slx 输出采样或参数。');
end

%% 为扫描预分配
Nt = length(tdown_list);
Nu = length(tup_list);

% 用 NaN 填充矩阵（方便 mask 无效组合）
theta1_map = nan(Nt, Nu);
theta2_map = nan(Nt, Nu);
theta3_map = nan(Nt, Nu);
RMSE_map   = nan(Nt, Nu);
fit_map    = nan(Nt, Nu);

% 为展示，保存若干示例仿真结果（选取几个典型组合）
example_pairs = []; % 我们稍后把若干 (i,j) 加入此数组用于时域绘图
angle_sims_cell = cell(Nt, Nu);  % 若内存允许可保存每次仿真结果的 angle 否则注释掉

%% 主循环：对所有 tdown/tup 组合进行辨识与仿真
fprintf('开始扫描 tdown 与 tup 的组合 ...\n');
for i = 1:Nt
    tdown = tdown_list(i);
    for j = 1:Nu
        tup = tup_list(j);
        % 只考虑合法组合： tup > tdown 且 tup <= time
        if tup <= tdown || tup > time
            continue;
        end

        % 计算索引（四舍五入到最近采样点）
        kstart = round(tdown/dt) + 1;
        kend   = round(tup/dt) + 1;
        nseg   = kend - kstart + 1;
        if nseg < 3
            % 段太短无法辨识（至少需要与未知参数数目相当的数据点）
            continue;
        end

        % 从先前滤波结果中直接取出该段信号（保持滤波一致）
        MotorAcc_F = -outFilter.Yt(kstart:kend)';  % 注意原代码有负号
        angle_F    = outFilter.x1(kstart:kend)';
        angledot_F = outFilter.x2(kstart:kend)';
        angleddot_F= outFilter.x3(kstart:kend)';

        % 组建回归矩阵并做最小二乘辨识（用 pinv 更稳健）
        phi_M = [angleddot_F' angledot_F' angle_F'];
        theta = pinv(phi_M) * MotorAcc_F';   % 3x1 向量
        if any(isnan(theta))
            continue;
        end

        % 存储 theta
        theta1_map(i,j) = theta(1);
        theta2_map(i,j) = theta(2);
        theta3_map(i,j) = theta(3);

        % 构造 Gs（供 Sim.slx 使用），放在 workspace
        Gs = tf(-1, [theta(1) theta(2) theta(3)]); %#ok<NASGU>

        % 运行被辨识系统/仿真（Sim.slx 从 current workspace 读取 Gs）
        outSim = sim('Sim.slx', 'SrcWorkspace','current');

        % 计算残差与指标（对全时域或仅对辨识段？这里使用全时域与实测比较）
        residuals = outSim.angle - angle_data;
        RMSEs = sqrt(mean(residuals.^2));
        RMSE_map(i,j) = RMSEs;
        fit_map(i,j) = 100 * (1 - norm(residuals) / norm(angle_data - mean(angle_data)));

        % 保存仿真角度（可用于后续可视化）
        angle_sims_cell{i,j} = outSim.angle;

        % 记录若干典型组合用于后续时域对比（例如：第一个合法、最后一个、以及中位）
        if isempty(example_pairs)
            example_pairs = [example_pairs; i j];
        elseif size(example_pairs,1) < 4
            example_pairs = [example_pairs; i j];
        end
    end
end

%% ---------------- 绘图：热图显示 theta 与 RMSE 随截取区间变化 ----------------
% 注意： x 轴为 tup, y 轴为 tdown（以秒为单位）
[X_u, Y_d] = meshgrid(tup_list, tdown_list);

figure('Name','辨识参数 θ 与 RMSE 随 tdown/tup 的变化','Units','normalized','Position',[0.05 0.05 0.9 0.8]);

subplot(2,2,1);
imagesc(tup_list, tdown_list, theta1_map);
axis xy; colorbar;
xlabel('tup (s)'); ylabel('tdown (s)');
title('\theta_1 (map)');

subplot(2,2,2);
imagesc(tup_list, tdown_list, theta2_map);
axis xy; colorbar;
xlabel('tup (s)'); ylabel('tdown (s)');
title('\theta_2 (map)');

subplot(2,2,3);
imagesc(tup_list, tdown_list, theta3_map);
axis xy; colorbar;
xlabel('tup (s)'); ylabel('tdown (s)');
title('\theta_3 (map)');

subplot(2,2,4);
imagesc(tup_list, tdown_list, RMSE_map);
axis xy; colorbar;
xlabel('tup (s)'); ylabel('tdown (s)');
title('RMSE (map)');

% 可选：将 NaN 区域用灰色标注（更清晰）
colormap('parula');

%% ---------------- 绘图：若干典型截取区间的时域对比 ----------------
% 选择 up to 4 示例组合（如果 example_pairs 留空则选用默认 tdown=1,tup=5）
if isempty(example_pairs)
    % default
    idx_default_i = find(tdown_list==1,1);
    idx_default_j = find(tup_list==5,1);
    if ~isempty(idx_default_i) && ~isempty(idx_default_j)
        example_pairs = [idx_default_i idx_default_j];
    else
        % 退化：选第一个合法组合
        [ii,jj] = find(~isnan(theta1_map),1);
        example_pairs = [ii jj];
    end
end

figure('Name','时域对比：若干截取段的仿真 vs 实测','Units','normalized','Position',[0.1 0.1 0.8 0.6]);
plot(t, angle_data, 'k', 'LineWidth', 1.6); hold on;
colors = lines(size(example_pairs,1));
legend_entries = {'实测角度'};
for p = 1:size(example_pairs,1)
    ii = example_pairs(p,1);
    jj = example_pairs(p,2);
    sim_angle = angle_sims_cell{ii,jj};
    if isempty(sim_angle)
        continue;
    end
    plot(t, sim_angle, 'LineWidth', 1.2);
    legend_entries{end+1} = sprintf('sim tdown=%.2f tup=%.2f', tdown_list(ii), tup_list(jj));
end
legend(legend_entries,'Location','best');
xlabel('时间 (s)');
ylabel('角度 (rad)');
title('不同截取段的辨识模型仿真对比（与实测）');
grid on;

%% ---------------- 输出最优片段建议（按 RMSE 最小） ----------------
[minRMSE, ind] = min(RMSE_map(:));
if ~isempty(ind) && ~isnan(minRMSE)
    [ii,jj] = ind2sub(size(RMSE_map), ind);
    fprintf('RMSE 最小值 %.5g，对应 tdown=%.2f s, tup=%.2f s\n', minRMSE, tdown_list(ii), tup_list(jj));
    fprintf('对应参数 theta = [%.6g, %.6g, %.6g]\n', theta1_map(ii,jj), theta2_map(ii,jj), theta3_map(ii,jj));
else
    fprintf('未找到有效的 RMSE 结果，请检查 tdown/tup 列表或滤波/仿真设置。\n');
end

