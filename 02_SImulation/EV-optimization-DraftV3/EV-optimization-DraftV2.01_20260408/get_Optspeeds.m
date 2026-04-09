function OPTout = get_Optspeeds(Pred_speds, param, degmap, batt, alpha)
%GET_OPTSPEEDS (WITH EV MODEL) - NORMALIZED
% - Prediction-side only (does NOT change real batt outside)
% - Internally clones batt into batt_pred/batt_opt
% - Outputs:
%   OPTout.pred.v_cmd : baseline command (Np) for replay-virtual compare
%   OPTout.opt.v      : optimal command (Np) for replay apply

% -------------------------
% 1) sizes
% -------------------------
Nx  = size(Pred_speds,1);
len = Nx - 1;
Np  = min(param.N_pred, len);

dist_init = Pred_speds(1,4);

dt_vec = Pred_speds(1:len,5);
bad = (~isfinite(dt_vec) | dt_vec <= 0);
if any(bad), dt_vec(bad) = param.dt; end

% -------------------------
% 2) bounds + baseline (from Pred_speds)
% -------------------------
[v_max, v_min, a_max, a_min, v_tot, a_tot] = get_bounds(Pred_speds, param);
v_tot = v_tot(:);
a_tot = a_tot(:);

% ensure v_tot length Nx (defensive)
if numel(v_tot) < Nx
    v_tot = [v_tot; repmat(v_tot(end), Nx-numel(v_tot), 1)];
else
    v_tot = v_tot(1:Nx);
end

% ensure a_tot length len (then expand to Nx later)
if numel(a_tot) < len
    a_tot = [a_tot; repmat(a_tot(end), len-numel(a_tot), 1)];
else
    a_tot = a_tot(1:len);
end

lb_v = v_min(2:Np+1);
ub_v = v_max(2:Np+1);

lb_a = a_min(1:Np);
ub_a = a_max(1:Np);

% baseline distance ref
dist_rel_end = 0;
for i = 1:len
    dist_rel_end = dist_rel_end + v_tot(i) * dt_vec(i);
end

% -------------------------
% 3) MPC solve (decision x = v2..v_{Np+1})
% -------------------------
MPCout = get_MPCsolver(param, Pred_speds, v_tot, dist_rel_end, ...
                       lb_v, ub_v, lb_a, ub_a, degmap, dist_init, batt, alpha);

opt_v_raw = MPCout.opt_v(:);

% enforce exact Np command for replay
if isempty(opt_v_raw)
    opt_v_cmd = v_tot(2:Np+1);
else
    if numel(opt_v_raw) >= Np
        opt_v_cmd = opt_v_raw(1:Np);
    else
        opt_v_cmd = [opt_v_raw; repmat(opt_v_raw(end), Np-numel(opt_v_raw), 1)];
    end
end

% baseline command for replay-virtual compare (Np)
pred_v_cmd = v_tot(2:Np+1);

% Nx-aligned speed for plotting / EV rollout
opt_v_plot = zeros(Nx,1);
opt_v_plot(1) = v_tot(1);
opt_v_plot(2:Np+1) = opt_v_cmd(1:Np);
if Np+1 < Nx
    opt_v_plot(Np+2:end) = opt_v_cmd(end);
end

% Nx-aligned dt / pos axis
pos_axis = Pred_speds(:,4);
dt_axis  = [dt_vec; dt_vec(end)];

% -------------------------
% 4) EV rollouts (Nx aligned)
% -------------------------
pred_ev = init_ev_series(Nx);
opt_ev  = init_ev_series(Nx);

pred_dist_rel = zeros(Nx,1);
opt_dist_rel  = zeros(Nx,1);

% ---- batt clones (IMPORTANT) ----
batt_pred = batt;
batt_opt  = batt;

% ---- baseline acc (Nx) ----
a_base = a_tot(:);
a_base = [a_base; a_base(end)]; % make length len+1
if numel(a_base) < Nx
    a_base = [a_base; repmat(a_base(end), Nx-numel(a_base),1)];
else
    a_base = a_base(1:Nx);
end

% ---- optimized acc (Nx) derived from v ----
a_opt = zeros(Nx,1);
for k = 1:len
    a_opt(k) = (opt_v_plot(k+1) - opt_v_plot(k)) / dt_vec(k);
end
a_opt(Nx) = a_opt(Nx-1);

% ---- rollout ----
for i = 1:Nx

    if i >= 2
        pred_dist_rel(i) = pred_dist_rel(i-1) + v_tot(i-1)     * dt_vec(i-1);
        opt_dist_rel(i)  = opt_dist_rel(i-1)  + opt_v_plot(i-1)* dt_vec(i-1);
    end

    param_i = param;
    if i <= len
        param_i.dt = dt_vec(i);
    else
        param_i.dt = dt_vec(end);
    end

    [EVp, batt_pred] = get_evmodel(v_tot(i), a_base(i), ...
        pred_dist_rel(i), dist_init, param_i, degmap, batt_pred);

    pred_ev = put_ev_step(pred_ev, i, EVp);

    [EVo, batt_opt] = get_evmodel(opt_v_plot(i), a_opt(i), ...
        opt_dist_rel(i), dist_init, param_i, degmap, batt_opt);

    opt_ev = put_ev_step(opt_ev, i, EVo);
end

% -------------------------
% 5) pack
% -------------------------
OPTout = struct();

% commands
OPTout.pred.v_cmd = pred_v_cmd;     % Np 
OPTout.opt.v      = opt_v_cmd;      % Np 

% axes
OPTout.axis.pos = pos_axis;         % Nx
OPTout.axis.dt  = dt_axis;          % Nx
OPTout.axis.Nx  = Nx;
OPTout.axis.Np  = Np;

% prediction (baseline)
OPTout.pred.v      = v_tot;
OPTout.pred.acc    = a_base;
OPTout.pred = get_pred_data(OPTout.pred, pred_ev);
OPTout.pred.batt_end = batt_pred;

% prediction (opt)
OPTout.opt.v_plot  = opt_v_plot;    % Nx aligned for plot
OPTout.opt.acc     = a_opt;
OPTout.opt = get_pred_data(OPTout.opt, opt_ev);
OPTout.opt.batt_end = batt_opt;

% bounds
OPTout.bound.v_max = v_max;
OPTout.bound.v_min = v_min;
OPTout.bound.lb_v  = lb_v;
OPTout.bound.ub_v  = ub_v;
OPTout.bound.a_max = a_max;
OPTout.bound.a_min = a_min;
OPTout.bound.lb_a  = lb_a;
OPTout.bound.ub_a  = ub_a;

% solver/meta
OPTout.mpc = MPCout;
OPTout.meta.dist_init = dist_init;
end

function S = init_ev_series(n)
% get_Optspeeds 전용 EV 시계열 구조체 초기화
fields = get_pred_hist();
S = struct();
for i = 1:numel(fields)
    S.(fields{i}) = nan(n,1);
end
end

function S = put_ev_step(S, k, ev)
% get_Optspeeds 전용 EV 단일 스텝 저장
fields = get_pred_hist();
for i = 1:numel(fields)
    name = fields{i};
    S.(name)(k,1) = getfield_safe(ev, name, NaN);
end
end

function v = getfield_safe(S, name, default)
% 필드가 없거나 비어 있으면 기본값 반환
v = default;
if isstruct(S) && isfield(S,name) && ~isempty(S.(name))
    v = S.(name);
end
end
