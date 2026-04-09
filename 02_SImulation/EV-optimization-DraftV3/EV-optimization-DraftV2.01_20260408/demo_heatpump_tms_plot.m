function demo_heatpump_tms_plot()
%DEMO_HEATPUMP_TMS_PLOT Simple BTMS demo with Np = 2 and P_tms plotting.

close all;

param = get_parameters(1800);
param.Np = 2;
param.Npar = param.Np;

% Mild cold-start scenario to activate battery heating without staying saturated.
param.T_amb = 5 + 273.15;
param.T_init = 10 + 273.15;
param.T_batt_min = 0 + 273.15;
param.T_ref = 25 + 273.15;

degmap = table2array(readtable('StudyArea_SlopeMap.csv'));
history = get_HistoryInit();

batt_app = init_batt(param);
batt_base = init_batt(param);

N = 1500;
t = (0:N-1)' * param.dt;
v_app = speed_profile(t);
v_base = min(v_app * 1.08 + 0.4, 25);

dist_app = 0;
dist_base = 0;

for k = 1:N
    if k < N
        acc_app = (v_app(k+1) - v_app(k)) / param.dt;
        acc_base = (v_base(k+1) - v_base(k)) / param.dt;
    else
        acc_app = 0;
        acc_base = 0;
    end

    if k > 1
        dist_app = dist_app + v_app(k-1) * param.dt;
        dist_base = dist_base + v_base(k-1) * param.dt;
    end

    [ev_app, batt_app] = get_evmodel(v_app(k), acc_app, dist_app, 0, param, degmap, batt_app);
    history = append_motion(history, 'app', k, t(k), dist_app, v_app(k), acc_app);
    history = get_appd_hist(history, 'app', ev_app);

    [ev_base, batt_base] = get_evmodel(v_base(k), acc_base, dist_base, 0, param, degmap, batt_base);
    history = append_motion(history, 'base', k, t(k), dist_base, v_base(k), acc_base);
    history = get_appd_hist(history, 'base', ev_base);
end

get_plot(history, param, 'tms_only');

outdir = fullfile(pwd, 'fig_results');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

fig = findobj('Type', 'figure', 'Name', 'P_tms');
if ~isempty(fig)
    exportgraphics(fig(1), fullfile(outdir, 'P_tms_heatpump_Np2.png'), 'Resolution', 150);
end
save(fullfile(outdir, 'P_tms_heatpump_Np2.mat'), 'history', 'param');
end

function batt = init_batt(param)
batt = struct();
batt.Q_bat = param.Q_bat0;
batt.Q_Ah = param.Q_bat0;
batt.I_prev = 0;
batt.Ah_th = 0;
batt.Ah_eff = 0;
batt.T = param.T_init;
batt.Rb = param.R_pack0;
batt.R_str0 = param.R_pack0;
end

function history = append_motion(history, group, step_k, time_k, dist_k, v_k, a_k)
history = append_scalar(history, group, 'step', step_k);
history = append_scalar(history, group, 't', time_k);
history = append_scalar(history, group, 'pos', dist_k);
history = append_scalar(history, group, 'v', v_k);
history = append_scalar(history, group, 'spd', v_k);
history = append_scalar(history, group, 'acc', a_k);
end

function v = speed_profile(t)
v = zeros(size(t));

idx1 = t < 20;
idx2 = t >= 20 & t < 80;
idx3 = t >= 80 & t < 140;
idx4 = t >= 140 & t < 220;
idx5 = t >= 220;

v(idx1) = 0.5 * t(idx1);
v(idx2) = 10 + 2 * sin(0.08 * (t(idx2) - 20));
v(idx3) = 12 + 4 * sin(0.05 * (t(idx3) - 80));
v(idx4) = 8 + 1.5 * sin(0.10 * (t(idx4) - 140));
v(idx5) = max(2, 8 - 0.12 * (t(idx5) - 220));
end

function history = append_scalar(history, group, name, value)
if ~isfield(history.(group), name) || isempty(history.(group).(name))
    history.(group).(name) = value;
else
    history.(group).(name)(end+1,1) = value;
end
end
