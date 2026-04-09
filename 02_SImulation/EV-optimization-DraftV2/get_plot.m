function get_plot(history, param)

close all;
lw = 1.5; fs = 14; fn = 'Times New Roman';

% -----------------------------
% 1) Time axis (based on applied)
% -----------------------------
N = 0;
if isfield(history,'applied_step') && ~isempty(history.applied_step)
    N = numel(history.applied_step);
elseif isfield(history,'applied_v') && ~isempty(history.applied_v)
    N = numel(history.applied_v);
elseif isfield(history,'applied_spd') && ~isempty(history.applied_spd)
    N = numel(history.applied_spd);
end
if N <= 0
    warning('get_plot: no applied data.');
    return;
end

if isfield(history,'applied_t') && numel(history.applied_t) == N && all(isfinite(history.applied_t))
    t = history.applied_t(:);
else
    dt = param.dt;
    t = (0:N-1)' * dt;
end

% -----------------------------
% 2) Pull vectors
% -----------------------------
% Applied 
app_v_cmd = getvec(history,'applied_v');     % command
app_v_mea = getvec(history,'applied_spd');   % measured speed
app_a     = getvec(history,'applied_acc');
app_pos   = getvec(history,'applied_pos');
app_P     = getvec(history,'applied_P_req');
app_I     = getvec(history,'applied_I_batt');
app_QAh   = getvec(history,'applied_Q_Ah');
app_SOC   = getvec(history,'applied_SOC');
app_Qbatt = getvec(history,'applied_Q_batt');
app_Rin   = getvec(history,'applied_Rin');
app_Qloss = getvec(history,'applied_Q_loss');

% Base 
base_v   = getvec(history,'base_v');
base_a   = getvec(history,'base_acc');
base_P   = getvec(history,'base_P_req');
base_I   = getvec(history,'base_I_batt');
base_QAh = getvec(history,'base_Q_Ah');
base_SOC = getvec(history,'base_SOC');
base_pos = getvec(history,'base_pos');

% Bounds 
vmin = getvec(history,'v_lb');
vmax = getvec(history,'v_ub');
amin = getvec(history,'a_lb');
amax = getvec(history,'a_ub');

% -----------------------------
% 3) Align lengths to N
% -----------------------------
app_v_cmd = fitN(app_v_cmd, N);
app_v_mea = fitN(app_v_mea, N);
app_a     = fitN(app_a, N);
app_pos   = fitN(app_pos, N);
app_P     = fitN(app_P, N);
app_I     = fitN(app_I, N);
app_QAh   = fitN(app_QAh, N);
app_SOC   = fitN(app_SOC, N);
app_Qbatt = fitN(app_Qbatt, N);
app_Rin   = fitN(app_Rin, N);
app_Qloss = fitN(app_Qloss, N);

base_v   = fitN(base_v, N);
base_a   = fitN(base_a, N);
base_P   = fitN(base_P, N);
base_I   = fitN(base_I, N);
base_QAh = fitN(base_QAh, N);
base_SOC = fitN(base_SOC, N);

vmin = fitN(vmin, N);
vmax = fitN(vmax, N);
amin = fitN(amin, N);
amax = fitN(amax, N);

app_v = app_v_mea;
if all(~isfinite(app_v))
    app_v = app_v_cmd;
end

% -----------------------------
% 4) Plots
% -----------------------------

% 1) Speed + bounds
figure('Name','Speed','Color','w'); hold on; grid on; box on;
plot(t, base_v*3.6, '--', 'LineWidth', lw);
plot(t, app_v*3.6, '-', 'LineWidth', lw);
plot(t, vmin*3.6, ':', 'LineWidth', 1.2);
plot(t, vmax*3.6, ':', 'LineWidth', 1.2);
legend('Base','Optimal','v_{min}','v_{max}','Location','best');
xlabel('Time [s]'); ylabel('Speed [km/h]');
title('Speed'); set(gca,'FontSize',fs,'FontName',fn);

% 2) Accel + bounds
figure('Name','Acceleration','Color','w'); hold on; grid on; box on;
plot(t, base_a, '--', 'LineWidth', lw);
plot(t, app_a,  '-', 'LineWidth', lw);
plot(t, amin,   ':', 'LineWidth', 1.2);
plot(t, amax,   ':', 'LineWidth', 1.2);
legend('Base','Optimal','a_{min}','a_{max}','Location','best');
xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
title('Acceleration'); set(gca,'FontSize',fs,'FontName',fn);

% 3) Distance
if any(isfinite(app_pos))
    figure('Name','Distance','Color','w'); hold on; grid on; box on;
    plot(t, base_pos/1000, '--','LineWidth', lw);
    plot(t, app_pos/1000, '-', 'LineWidth', lw);
    legend('Base','Optimal','Location','best');
    xlabel('Time [s]'); ylabel('Position [km]');
    title('Distance'); set(gca,'FontSize',fs,'FontName',fn);
end

% 4) Power
figure('Name','Power','Color','w'); hold on; grid on; box on;
plot(t, base_P/1000, '--', 'LineWidth', lw);
plot(t, app_P/1000,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('Power [kW]');
title('Battery Power'); set(gca,'FontSize',fs,'FontName',fn);

% 5) Current
figure('Name','Current','Color','w'); hold on; grid on; box on;
plot(t, base_I, '--', 'LineWidth', lw);
plot(t, app_I,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('Current [A]');
title('Battery Current'); set(gca,'FontSize',fs,'FontName',fn);

% 6) Capacity (Ah)
figure('Name','Capacity','Color','w'); hold on; grid on; box on;
plot(t, base_QAh, '--', 'LineWidth', lw);
plot(t, app_QAh,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('Q [Ah]');
title('Battery Capacity (Coulomb Count)'); set(gca,'FontSize',fs,'FontName',fn);

% 7) SOC
figure('Name','SOC','Color','w'); hold on; grid on; box on;
plot(t, base_SOC, '--', 'LineWidth', lw);
plot(t, app_SOC,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('SOC [%]');
title('SOC'); set(gca,'FontSize',fs,'FontName',fn);

% 8) Q_batt 
if any(isfinite(app_Qbatt))
    figure('Name','Q_batt','Color','w'); hold on; grid on; box on;
    plot(t, app_Qbatt, '-', 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('Q_{batt} [W]');
    title('Battery Joule Heat'); set(gca,'FontSize',fs,'FontName',fn);
end

% 9) Rin 
if any(isfinite(app_Rin))
    figure('Name','Rin','Color','w'); hold on; grid on; box on;
    plot(t, app_Rin, '-', 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('R_{in} [Ohm]');
    title('Battery Internal Resistance'); set(gca,'FontSize',fs,'FontName',fn);
end

% 10) Q_loss 
if any(isfinite(app_Qloss))
    figure('Name','Q_loss','Color','w'); hold on; grid on; box on;
    plot(t, app_Qloss, '-', 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('Q_{loss}');
    title('Battery Capacity Loss [%]'); set(gca,'FontSize',fs,'FontName',fn);
end

end

% =========================
% helpers
% =========================
function x = getvec(S, name)
x = [];
if isstruct(S) && isfield(S,name) && ~isempty(S.(name))
    x = S.(name)(:);
end
end

function y = fitN(x, N)
if isempty(x)
    y = nan(N,1);
    return;
end
x = x(:);
if numel(x) >= N
    y = x(1:N);
else
    y = [x; nan(N-numel(x),1)];
end
end
