function get_plot(history, param, plot_mode)

if nargin < 3 || isempty(plot_mode)
    plot_mode = 'all';
end

plot_only_tms = strcmpi(plot_mode, 'tms_only');

close all;
lw = 1.5;
fs = 14;
fn = 'Times New Roman';

if ~isfield(history,'app') || ~isstruct(history.app)
    warning('get_plot: no app data.');
    return;
end

app = history.app;
base = get_group(history, 'base');

N = get_N(app);
if N <= 0
    warning('get_plot: no app data.');
    return;
end

if isfield(app,'t') && numel(app.t) == N && all(isfinite(app.t))
    t = app.t(:);
else
    t = (0:N-1)' * param.dt;
end

app_v_cmd = fitN(getvec(app,'v'), N);
app_v_mea = fitN(getvec(app,'spd'), N);
app_a     = fitN(getvec(app,'acc'), N);
app_pos   = fitN(getvec(app,'pos'), N);
app_P     = fitN(getvec(app,'P_req'), N);
app_I     = fitN(getvec(app,'I_batt'), N);
app_QAh   = fitN(getvec(app,'Q_Ah'), N);
app_SOC   = fitN(getvec(app,'SOC'), N);
app_Qbatt = fitN(getvec(app,'Q_batt'), N);
app_Ahir  = fitN(getvec(app,'Ah_ir'), N);
app_Rin   = fitN(getvec(app,'Rin'), N);
app_Qloss = fitN(getvec(app,'Q_loss'), N);
app_AgeCost = fitN(getvec(app,'AgeCost'), N);
app_Tbatt = fitN(getvec(app,'T_batt'), N);
app_Tref  = fitN(getvec(app,'T_ref'), N);
app_Pcool = fitN(getvec(app,'P_cool'), N);
app_Pheat = fitN(getvec(app,'P_heat'), N);
app_Ptms  = fitN(getvec(app,'P_tms'), N);
base_v     = fitN(getvec(base,'v'), N);
base_a     = fitN(getvec(base,'acc'), N);
base_P     = fitN(getvec(base,'P_req'), N);
base_I     = fitN(getvec(base,'I_batt'), N);
base_QAh   = fitN(getvec(base,'Q_Ah'), N);
base_SOC   = fitN(getvec(base,'SOC'), N);
base_pos   = fitN(getvec(base,'pos'), N);
base_Ahir  = fitN(getvec(base,'Ah_ir'), N);
base_AgeCost = fitN(getvec(base,'AgeCost'), N);
base_Tbatt = fitN(getvec(base,'T_batt'), N);
base_Pcool = fitN(getvec(base,'P_cool'), N);
base_Pheat = fitN(getvec(base,'P_heat'), N);
base_Ptms  = fitN(getvec(base,'P_tms'), N);

vmin = fitN(getvec(app,'v_lb'), N);
vmax = fitN(getvec(app,'v_ub'), N);
amin = fitN(getvec(app,'a_lb'), N);
amax = fitN(getvec(app,'a_ub'), N);

app_v = app_v_mea;
if all(~isfinite(app_v))
    app_v = app_v_cmd;
end

if all(~isfinite(app_Ptms))
    app_Ptms = nz(app_Pcool) + nz(app_Pheat);
end
if all(~isfinite(base_Ptms))
    base_Ptms = nz(base_Pcool) + nz(base_Pheat);
end

if plot_only_tms
    plot_tms_figure(t, base_Ptms, app_Ptms, lw, fs, fn);
    return;
end

figure('Name','Speed','Color','w'); hold on; grid on; box on;
plot(t, base_v*3.6, '--', 'LineWidth', lw);
plot(t, app_v*3.6, '-', 'LineWidth', lw);
plot(t, vmin*3.6, ':', 'LineWidth', 1.2);
plot(t, vmax*3.6, ':', 'LineWidth', 1.2);
legend('Base','Optimal','v_{min}','v_{max}','Location','best');
xlabel('Time [s]'); ylabel('Speed [km/h]');
title('Speed'); set(gca,'FontSize',fs,'FontName',fn);

figure('Name','Acceleration','Color','w'); hold on; grid on; box on;
plot(t, base_a, '--', 'LineWidth', lw);
plot(t, app_a,  '-', 'LineWidth', lw);
plot(t, amin,   ':', 'LineWidth', 1.2);
plot(t, amax,   ':', 'LineWidth', 1.2);
legend('Base','Optimal','a_{min}','a_{max}','Location','best');
xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
title('Acceleration'); set(gca,'FontSize',fs,'FontName',fn);

if any(isfinite(app_pos))
    figure('Name','Distance','Color','w'); hold on; grid on; box on;
    plot(t, base_pos/1000, '--','LineWidth', lw);
    plot(t, app_pos/1000, '-', 'LineWidth', lw);
    legend('Base','Optimal','Location','best');
    xlabel('Time [s]'); ylabel('Position [km]');
    title('Distance'); set(gca,'FontSize',fs,'FontName',fn);
end

figure('Name','Power','Color','w'); hold on; grid on; box on;
plot(t, base_P/1000, '--', 'LineWidth', lw);
plot(t, app_P/1000,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('Power [kW]');
title('Battery Power'); set(gca,'FontSize',fs,'FontName',fn);

figure('Name','Current','Color','w'); hold on; grid on; box on;
plot(t, base_I, '--', 'LineWidth', lw);
plot(t, app_I,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('Current [A]');
title('Battery Current'); set(gca,'FontSize',fs,'FontName',fn);

figure('Name','Capacity','Color','w'); hold on; grid on; box on;
plot(t, base_QAh, '--', 'LineWidth', lw);
plot(t, app_QAh,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('Q [Ah]');
title('Battery Capacity (Coulomb Count)'); set(gca,'FontSize',fs,'FontName',fn);

figure('Name','SOC','Color','w'); hold on; grid on; box on;
plot(t, base_SOC, '--', 'LineWidth', lw);
plot(t, app_SOC,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('SOC [%]');
title('SOC'); set(gca,'FontSize',fs,'FontName',fn);

if any(isfinite(app_Qbatt))
    figure('Name','Q_batt','Color','w'); hold on; grid on; box on;
    plot(t, app_Qbatt, '-', 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('Q_{batt} [W]');
    title('Battery Joule Heat'); set(gca,'FontSize',fs,'FontName',fn);
end

if any(isfinite(app_Rin))
    figure('Name','Rin','Color','w'); hold on; grid on; box on;
    plot(t, app_Rin, '-', 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('R_{in} [Ohm]');
    title('Battery Internal Resistance'); set(gca,'FontSize',fs,'FontName',fn);
end

if any(isfinite(app_Ahir))
    figure('Name','Ah_ir','Color','w'); hold on; grid on; box on;
    plot(t, base_Ahir, '--', 'LineWidth', lw);
    plot(t, app_Ahir, '-', 'LineWidth', lw);
    legend('Base','Optimal','Location','best');
    xlabel('Time [s]'); ylabel('Throughput [Ah]');
    title('Accumulated Current Throughput'); set(gca,'FontSize',fs,'FontName',fn);
end

if any(isfinite(app_Qloss))
    figure('Name','Q_loss','Color','w'); hold on; grid on; box on;
    plot(t, app_Qloss, '-', 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('Q_{loss}');
    title('Battery Capacity Loss [%]'); set(gca,'FontSize',fs,'FontName',fn);
end

if any(isfinite(app_AgeCost))
    figure('Name','AgeCost','Color','w'); hold on; grid on; box on;
    plot(t, base_AgeCost, '--', 'LineWidth', lw);
    plot(t, app_AgeCost, '-', 'LineWidth', lw);
    legend('Base','Optimal','Location','best');
    xlabel('Time [s]'); ylabel('AgeCost');
    title('Instantaneous Aging Cost'); set(gca,'FontSize',fs,'FontName',fn);
end

if any(isfinite(app_Tbatt))
    figure('Name','Temperature','Color','w'); hold on; grid on; box on;
    plot(t, base_Tbatt - 273.15, '--', 'LineWidth', lw);
    plot(t, app_Tbatt - 273.15,  '-', 'LineWidth', lw);
    if any(isfinite(app_Tref))
        plot(t, app_Tref - 273.15, ':', 'LineWidth', 1.2);
    end
    legend('Base','Optimal','T_{ref}','Location','best');
    xlabel('Time [s]'); ylabel('Temperature [degC]');
    title('Battery Temperature'); set(gca,'FontSize',fs,'FontName',fn);
end

if any(isfinite(app_Ptms))
    plot_tms_figure(t, base_Ptms, app_Ptms, lw, fs, fn);
end

end

function plot_tms_figure(t, base_Ptms, app_Ptms, lw, fs, fn)
figure('Name','P_tms','Color','w', 'ToolBar', 'none'); hold on; grid on; box on;
plot(t, base_Ptms/1000, '--', 'LineWidth', lw);
plot(t, app_Ptms/1000,  '-', 'LineWidth', lw);
legend('Base','Optimal','Location','best');
xlabel('Time [s]'); ylabel('P_{tms} [kW]');
title('Thermal Management Power'); set(gca,'FontSize',fs,'FontName',fn);
end

function S = get_group(history, name)
S = struct();
if isstruct(history) && isfield(history,name) && isstruct(history.(name))
    S = history.(name);
end
end

function N = get_N(app)
N = 0;
if isfield(app,'step') && ~isempty(app.step)
    N = numel(app.step);
elseif isfield(app,'v') && ~isempty(app.v)
    N = numel(app.v);
elseif isfield(app,'spd') && ~isempty(app.spd)
    N = numel(app.spd);
end
end

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

function x = nz(x)
x(~isfinite(x)) = 0;
end
