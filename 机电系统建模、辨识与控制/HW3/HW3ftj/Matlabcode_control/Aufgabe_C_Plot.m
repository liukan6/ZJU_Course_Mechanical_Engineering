clear; close all; clc;
CtrlResData_full = [];
for poleindex = 1:6
    load("Aufgabe_C\Aufgabe_C_CtrlResData_"+poleindex+".mat");
    CtrlResData_full = [CtrlResData_full CtrlResData];
end

bandwidths = [5, 15, 25, 5, 15, 25];   % 对应每个 case 的闭环带宽 (rad/s) 或标签
init_deg   = [6 6 6 10 10 10];         % 初始摆杆角度 (deg)

% prepare storage for summary metrics
Ncases = numel(CtrlResData_full);
CaseIdx = (1:Ncases).';
InitTheta_deg = NaN(Ncases,1);
Bandwidth = NaN(Ncases,1);
SettlingTime_2pct = NaN(Ncases,1);
Overshoot_pct = NaN(Ncases,1);
% SteadyStateError removed
ControlPeak = NaN(Ncases,1);
ControlRMS = NaN(Ncases,1);
RiseTime = NaN(Ncases,1);

% ---- loop cases, compute metrics, and plot case-wise figures ----
figure_handles = gobjects(Ncases,1);
for k = 1:Ncases
    C = CtrlResData_full(k);
    t = C.time(:);
    % signals: Yd (reference, deg), FedY (feedback, deg), uout (control), alpha (deg), theta (deg)
    ref = C.Yd(end);   % assume reference is final value (deg)
    y = C.FedY(:);
    u = C.uout(:);
    alpha = C.alpha(:);
    theta = C.theta(:);

    % metrics (angles are in degrees already)
    InitTheta_deg(k) = theta(1);    % already in deg
    Bandwidth(k) = bandwidths(k);
    SettlingTime_2pct(k) = settling_time_2pct(t, theta, ref);
    Overshoot_pct(k) = overshoot_pct(theta, ref);
    % SteadyStateError computation omitted by request
    ControlPeak(k) = max(abs(u));
    ControlRMS(k) = sqrt(mean(u.^2));
    RiseTime(k) = rise_time_10_90(t, theta, ref);

    % --- plot: 3 subplots in a single figure per case ---
    fh = figure('Name',sprintf('Case %d: bw=%g, init=%.1fdeg',k,bandwidths(k),init_deg(k)),'NumberTitle','off','Color','w');
    figure_handles(k) = fh;
    % 1) control input
    subplot(3,1,1);
    plot(t, u, 'LineWidth',1.2); grid on;
    ylabel('u_{out}'); xlabel('Time (s)');
    title(sprintf('Case %d -- control input', k));
    txt = sprintf('peak=%.3g, RMS=%.3g', ControlPeak(k), ControlRMS(k));
    xlim([t(1), t(end)]);
    legend(txt,'Location','best');

    % 2) table angle alpha (deg)
    subplot(3,1,2);
    plot(t, alpha, 'LineWidth',1.2); grid on;
    ylabel('\alpha (deg)'); xlabel('Time (s)');
    title('Table angle \alpha(t)');

    % 3) rod angle theta (deg) (plot reference also)
    subplot(3,1,3);
    plot(t, theta, 'b','LineWidth',1.2); hold on;
    plot(t, ref*ones(size(t)), 'r--','LineWidth',1);
    grid on; xlabel('Time (s)'); ylabel('\theta (deg)');
    title(sprintf('\\theta(t) (ref=%.4g deg). OS=%.2f%%, ts2%%=%.3g s', ref, Overshoot_pct(k), SettlingTime_2pct(k)));
    % remove annotation of steady state error (per request)

    % adjust figure layout
    set(fh,'Position',[200 50 700 800]);
    drawnow;
end

% ---- combined overlay plot for theta across all cases (for comparison) ----
fh_all = figure('Name','All cases: theta overlay (deg)','NumberTitle','off','Color','w');
colors = lines(Ncases);
hold on; grid on;
for k=1:Ncases
    plot(CtrlResData_full(k).time, CtrlResData_full(k).theta, 'Color', colors(k,:), 'LineWidth', 1.2);
end
legend_entries = arrayfun(@(i) sprintf('case%d: bw=%g, init=%g°', i, bandwidths(i), init_deg(i)), (1:Ncases),'UniformOutput',false);
legend(legend_entries,'Location','best');
xlabel('Time (s)'); ylabel('\theta (deg)');
title('Comparison of \theta(t) (deg) across all 6 cases');
set(fh_all,'Position',[100 100 900 400]);

% ---- combined overlay plot for alpha across all cases (for comparison) ----
fh_alpha = figure('Name','All cases: alpha overlay (deg)','NumberTitle','off','Color','w');
hold on; grid on;
for k = 1:Ncases
    tvec = CtrlResData_full(k).time;
    aval = CtrlResData_full(k).alpha;   % deg already
    plot(tvec, aval, 'Color', colors(k,:), 'LineWidth', 1.2);
end
legend(legend_entries,'Location','best','Interpreter','none');
xlabel('Time (s)');
ylabel('\alpha (deg)');
title('Comparison of \alpha(t) (deg) across all 6 cases');
set(fh_alpha,'Position',[100 100 900 400]);

% ---- summary table (angles in degrees), WITHOUT theta steady-state error column ----
T = table(CaseIdx, Bandwidth, InitTheta_deg, Overshoot_pct, RiseTime, SettlingTime_2pct, ControlPeak, ControlRMS, ...
    'VariableNames',{'Case','Bandwidth','InitTheta_deg','Overshoot_pct','RiseTime_s','SettlingTime2pct_s','ControlPeak','ControlRMS'});

disp('Summary table for 6 cases (angles in degrees, without theta e_{ss}):');
disp(T);

% Optionally save figures and table
saveFigures = false;
if saveFigures
    for k=1:Ncases
        saveas(figure_handles(k), sprintf('Case%d_results_deg_no_ess.png', k));
    end
    saveas(fh_all, 'AllCases_theta_overlay_deg_no_ess.png');
    saveas(fh_alpha, 'AllCases_alpha_overlay_deg_no_ess.png');
    writetable(T,'TaskC_summary_table_deg_no_ess.csv');
    fprintf('Saved figures and summary csv.\n');
end

% helper local functions (nested for access)
    function ts = settling_time_2pct(t, y, ref)
        % compute 2% settling time: smallest t_s such that for all t>=t_s, |y-ref|<=tol
        if abs(ref) < 1e-9
            tol = 0.02 * max(abs(y));    % fallback tolerance if ref is near zero
            if tol < 1e-6, tol = 1e-6; end
        else
            tol = 0.02 * abs(ref);
        end
        err = abs(y - ref);
        idx_exceed = find(err > tol);
        if isempty(idx_exceed)
            ts = 0; % already within tolerance
        else
            last_exceed = idx_exceed(end);
            if last_exceed >= length(t)
                ts = NaN; % never settles within the simulated time
            else
                ts = t(last_exceed + 1);
            end
        end
    end

    function os = overshoot_pct(y, ref)
        % compute overshoot in percent relative to ref
        if abs(ref) < 1e-9
            os = NaN;
            return;
        end
        mx = max(y);
        os = (mx - ref)/abs(ref) * 100;
        if os < 0, os = 0; end
    end

    function tr = rise_time_10_90(t, y, ref)
        % compute 10%-90% rise time for step to ref
        if abs(ref) < 1e-9
            tr = NaN; return;
        end
        y0 = y(1);
        y10 = y0 + 0.1*(ref - y0);
        y90 = y0 + 0.9*(ref - y0);
        idx10 = find((y - y0) .* (ref - y0) >= 0 & abs(y - y0) >= abs(y10 - y0), 1); % first >=10%
        idx90 = find((y - y0) .* (ref - y0) >= 0 & abs(y - y0) >= abs(y90 - y0), 1);
        if isempty(idx10) || isempty(idx90)
            tr = NaN;
        else
            tr = t(idx90) - t(idx10);
        end
    end