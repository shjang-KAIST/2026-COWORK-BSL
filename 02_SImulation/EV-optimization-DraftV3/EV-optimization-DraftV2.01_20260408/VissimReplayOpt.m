function [history, Fixed_steps, SIM_STOP_FLAG, batt] = VissimReplayOpt( ...
    Vissim, TargetID, param, OPTout, Fixed_steps, Simout, history, degmap, batt)

SIM_STOP_FLAG = false;
dt = param.dt;

fprintf('[VissimReplayOpt] Apply OPT for Np steps (open-loop) and update APP+BASE batteries...\n');

if ~isfield(OPTout,'opt') || ~isfield(OPTout.opt,'v') || isempty(OPTout.opt.v)
    error('VissimReplayOpt: OPTout.opt.v missing/empty');
end
v_opt_cmd = OPTout.opt.v(:);

v_base_cmd = [];
if isfield(OPTout,'pred') && isfield(OPTout.pred,'v_cmd') && ~isempty(OPTout.pred.v_cmd)
    v_base_cmd = OPTout.pred.v_cmd(:);
elseif isfield(OPTout,'pred') && isfield(OPTout.pred,'v') && numel(OPTout.pred.v) >= 2
    v_base_cmd = OPTout.pred.v(2) * ones(numel(v_opt_cmd),1);
end
if isempty(v_base_cmd)
    v_base_cmd = v_opt_cmd;
end

Np     = param.N_pred;
Napply = min([Np, numel(v_opt_cmd), numel(v_base_cmd)]);
if Napply <= 0
    SIM_STOP_FLAG = true;
    return;
end

history = get_hist_ready(history, batt);

track_enable = false;
halfBox_m = 50;
if isfield(param,'track_enable') && ~isempty(param.track_enable)
    track_enable = logical(param.track_enable);
end
if isfield(param,'track_halfBox_m') && ~isempty(param.track_halfBox_m)
    halfBox_m = double(param.track_halfBox_m);
end

simSpeedApply = 1;
if isfield(param,'sim_speed_apply') && ~isempty(param.sim_speed_apply)
    simSpeedApply = double(param.sim_speed_apply);
end
try
    set(Vissim.Simulation,'AttValue','SimSpeed', simSpeedApply);
catch
end

dist_init = NaN;
if isfield(OPTout,'meta') && isfield(OPTout.meta,'dist_init') && ~isempty(OPTout.meta.dist_init)
    dist_init = double(OPTout.meta.dist_init);
end

lb_v = [];
ub_v = [];
lb_a = [];
ub_a = [];
if isfield(OPTout,'bound')
    if isfield(OPTout.bound,'lb_v'), lb_v = OPTout.bound.lb_v(:); end
    if isfield(OPTout.bound,'ub_v'), ub_v = OPTout.bound.ub_v(:); end
    if isfield(OPTout.bound,'lb_a'), lb_a = OPTout.bound.lb_a(:); end
    if isfield(OPTout.bound,'ub_a'), ub_a = OPTout.bound.ub_a(:); end
end

NO_CAR_DIST = 50;
if isfield(param,'NO_CAR_DIST') && ~isempty(param.NO_CAR_DIST)
    NO_CAR_DIST = double(param.NO_CAR_DIST);
end

for k = 1:Napply

    if Fixed_steps >= Simout
        SIM_STOP_FLAG = true;
        break;
    end

    try
        ego = Vissim.Net.Vehicles.ItemByKey(int32(TargetID));
    catch
        ego = [];
    end
    if isempty(ego)
        SIM_STOP_FLAG = true;
        break;
    end

    if track_enable
        get_TargetVehicle(Vissim, TargetID, halfBox_m);
    end

    v_des = v_opt_cmd(k);
    try
        set(ego,'AttValue','Speed', max(v_des,0)*3.6);
    catch
    end
    history.app.v(end+1,1) = v_des;
    history.app.lane(end+1,1) = NaN;

    ego_pos_meas = double(get(ego,'AttValue','Pos'));

    if ~isfinite(dist_init)
        dist_init = ego_pos_meas;
    end
    if ~isfinite(history.meta.pos0)
        history.meta.pos0 = ego_pos_meas;
    end
    if ~isfinite(history.meta.app_dist)
        history.meta.app_dist = ego_pos_meas - dist_init;
    end
    dist_rel_app = history.meta.app_dist;

    if ~isfinite(history.meta.app_v_prev)
        history.meta.app_v_prev = v_des;
    end
    opt_acc = (v_des - history.meta.app_v_prev) / dt;
    history.meta.app_v_prev = v_des;

    try
        ego_lane_str = char(get(ego,'AttValue','Lane'));
        ego_link = str2double(extractBefore(ego_lane_str,'-'));
        ego_lane = str2double(extractAfter(ego_lane_str,'-'));
        ego_len  = double(get(ego,'AttValue','Length'));

        LaneInfo = get_LaneInfo(Vissim, ego_pos_meas, ego_len, TargetID, ego_link, ego_lane, NO_CAR_DIST);
        history.app.f_dist(end+1,1) = LaneInfo(1).f_dist;
        history.app.r_dist(end+1,1) = LaneInfo(1).r_dist;
        history.app.f_v(end+1,1)    = LaneInfo(1).f_v;
        history.app.r_v(end+1,1)    = LaneInfo(1).r_v;
    catch
        ego_link = NaN;
        ego_lane = NaN;
        history.app.f_dist(end+1,1) = NaN;
        history.app.r_dist(end+1,1) = NaN;
        history.app.f_v(end+1,1)    = NaN;
        history.app.r_v(end+1,1)    = NaN;
    end
    history.app.link(end+1,1)    = ego_link;
    history.app.laneidx(end+1,1) = ego_lane;

    [EVout, batt] = get_evmodel(v_des, opt_acc, dist_rel_app, dist_init, param, degmap, batt);

    ego_pos_int = history.meta.pos0 + dist_rel_app;

    history.app.step(end+1,1) = Fixed_steps;
    history.app.t(end+1,1)    = Fixed_steps * dt;
    history.app.pos(end+1,1)  = ego_pos_int;
    history.app.spd(end+1,1)  = v_des;
    history.app.acc(end+1,1)  = opt_acc;
    history = get_appd_hist(history, 'app', EVout);

    history.app.v_lb(end+1,1) = getk(lb_v, k, NaN);
    history.app.v_ub(end+1,1) = getk(ub_v, k, NaN);
    history.app.a_lb(end+1,1) = getk(lb_a, k, NaN);
    history.app.a_ub(end+1,1) = getk(ub_a, k, NaN);

    history.meta.app_dist = history.meta.app_dist + v_des * dt;

    v_base = v_base_cmd(k);

    if ~isfinite(history.meta.base_v_prev)
        history.meta.base_v_prev = v_des;
    end
    a_base = (v_base - history.meta.base_v_prev) / dt;
    history.meta.base_v_prev = v_base;

    if ~isfinite(history.meta.base_dist)
        history.meta.base_dist = dist_rel_app;
    end
    dist_rel_base = history.meta.base_dist;

    [EVb, history.meta.base_batt] = get_evmodel(v_base, a_base, dist_rel_base, dist_init, param, degmap, history.meta.base_batt);

    base_pos_int = history.meta.pos0 + dist_rel_base;

    history.base.v(end+1,1)       = v_base;
    history.base.acc(end+1,1)     = a_base;
    history = get_appd_hist(history, 'base', EVb);
    history.base.pos_rel(end+1,1) = dist_rel_base;
    history.base.pos(end+1,1)     = base_pos_int;
    history.meta.base_dist = history.meta.base_dist + v_base * dt;

    try
        Vissim.Simulation.RunSingleStep;
    catch
        SIM_STOP_FLAG = true;
        break;
    end
    Fixed_steps = Fixed_steps + 1;
end

end

function history = get_hist_ready(history, batt)
if ~isfield(history,'app') || ~isstruct(history.app) || ~isfield(history.app,'v')
    history = get_HistoryInit();
end

if ~isfield(history,'meta') || ~isstruct(history.meta)
    history.meta = struct();
end
if ~isfield(history.meta,'base_batt') || isempty(history.meta.base_batt)
    history.meta.base_batt = batt;
end
if ~isfield(history.meta,'base_v_prev') || isempty(history.meta.base_v_prev)
    history.meta.base_v_prev = NaN;
end
if ~isfield(history.meta,'base_dist') || isempty(history.meta.base_dist)
    history.meta.base_dist = NaN;
end
if ~isfield(history.meta,'app_v_prev') || isempty(history.meta.app_v_prev)
    history.meta.app_v_prev = NaN;
end
if ~isfield(history.meta,'app_dist') || isempty(history.meta.app_dist)
    history.meta.app_dist = NaN;
end
if ~isfield(history.meta,'pos0') || isempty(history.meta.pos0)
    history.meta.pos0 = NaN;
end
end

function v = getk(x, k, default)
v = default;
if ~isempty(x) && numel(x) >= k && isfinite(x(k))
    v = x(k);
end
end
