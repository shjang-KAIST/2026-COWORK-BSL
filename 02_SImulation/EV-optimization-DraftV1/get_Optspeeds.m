function OPTout = get_Optspeeds(Pred_speds, Np, degmap)
% GET_OPTSPEEDS
%  - VISSIM에서 받은 Pred_speds(상태/환경 feature)로부터
%    (1) Baseline(현재 policy) 물리량 계산
%    (2) MPC로 speed profile 최적화
%    (3) 최적 결과를 다시 EV model로 post-process하여 OPTout struct로 패킹

%% 1) Load params & compute bounds
param = get_parameters();

% Bounds from traffic/spacing heuristic (v, a) + baseline rollout (total_v, total_a)
[up_buff_v, do_buff_v, up_buff_a, do_buff_a, total_v, total_a] = ...
    get_bounds(Pred_speds, param);

% Position feature: absolute position & local distance (w.r.t. current step)
Physical_pos = Pred_speds(1:Np+1, 4);
sys_dist     = Physical_pos - Pred_speds(1, 4);
dist_init = Pred_speds(1, 4);

len = size(Pred_speds,1) - 1;   % number of steps in Pred_speds horizon

%% 2) Baseline evaluation (no control optimization)
% Preallocate baseline signals
pred_P      = zeros(len,1);
pred_Trq    = zeros(len,1);
pred_RPM    = zeros(len,1);
pred_I_batt = zeros(len,1);

% Baseline: use (total_v, total_a) as given trajectory
for i = 1:len
    PredEV = get_evmodel(total_v(i), total_a(i), sys_dist(i), dist_init, param, degmap);

    pred_P(i)      = PredEV.P_total;
    pred_Trq(i)    = PredEV.trq;
    pred_RPM(i)    = PredEV.rpm;
    pred_I_batt(i) = PredEV.I_batt;
end

% Pad last sample for plotting alignment (len+1)
pred_P      = [pred_P; 0];
pred_Trq    = [pred_Trq; 0];
pred_RPM    = [pred_RPM; 0];
pred_I_batt = [pred_I_batt; 0];

%% 3) MPC solve (optimize speed profile)
% Reference distance: baseline distance over horizon
dist_ref = 0;
for i = 1:len
    dist_ref = dist_ref + total_v(i) * Pred_speds(i,5);
end

% Slice bounds to match decision variables:
%  - decision x = v(2..Np+1)  (v0 fixed outside)
lb_v = do_buff_v(2:Np+1);
ub_v = up_buff_v(2:Np+1);
lb_a = do_buff_a(1:Np);
ub_a = up_buff_a(1:Np);

% Solve MPC -> returns opt_v (Np+1) and opt_acc (Np)
MPCout = get_MPCsolver(param, Pred_speds, total_v, dist_ref, ...
                       lb_v, ub_v, lb_a, ub_a, degmap, dist_init);

opt_v   = MPCout.opt_v;
opt_acc = MPCout.opt_acc;

%% 4) Post-process (recompute physics precisely along optimized rollout)
% Allocate outputs (len)
opt_P_total = zeros(len,1);
opt_P_batt  = zeros(len,1);
opt_Q       = zeros(len,1);
opt_I       = zeros(len,1);
opt_Trq     = zeros(len,1);
opt_RPM     = zeros(len,1);

% Cumulative distance
cum_dist = zeros(len+1,1);

% Violation counters (for monitoring only)
vio_v = 0;
vio_a = 0;

cur_d = 0;
for i = 1:len
    dt = Pred_speds(i,5);

    % Use MPC result within horizon, otherwise hold last state
    if i <= Np
        v_curr = opt_v(i);
        acc    = opt_acc(i);
    else
        v_curr = opt_v(end);
        acc    = 0;
    end

    % Trapezoid distance update
    if i < len
        v_next_step = opt_v(min(i+1, length(opt_v)));
    else
        v_next_step = v_curr;
    end
    cur_d = cur_d + (v_curr + v_next_step)/2 * dt;
    cum_dist(i+1) = cur_d;

    % Recompute EV physics at step i
    OPTev = get_evmodel(v_curr, acc, cur_d, dist_init, param, degmap);

    opt_P_total(i) = OPTev.P_total;
    opt_P_batt(i)  = OPTev.P_batt;
    opt_Q(i)       = OPTev.Q_batt;
    opt_I(i)       = OPTev.I_batt;
    opt_Trq(i)     = OPTev.trq;
    opt_RPM(i)     = OPTev.rpm;

    % Simple violation check (tolerance 0.1)
    if i <= Np
        if v_curr < lb_v(i) - 0.1 || v_curr > ub_v(i) + 0.1, vio_v = vio_v + 1; end
        if acc    < lb_a(i) - 0.1 || acc    > ub_a(i) + 0.1, vio_a = vio_a + 1; end
    end
end

% Pad last sample for plotting alignment (len+1)
opt_P_total = [opt_P_total; 0];
opt_P_batt  = [opt_P_batt; 0];
opt_Q       = [opt_Q; 0];
opt_I       = [opt_I; 0];
opt_Trq     = [opt_Trq; 0];
opt_RPM     = [opt_RPM; 0];

% Absolute position (start pos + cum dist)
opt_pos = cum_dist + Pred_speds(1,4);

%% 5) Pack results
% Optimized trajectory
OPTout.opt.v       = opt_v;
OPTout.opt.acc     = opt_acc;
OPTout.opt.pos     = opt_pos;
OPTout.opt.P_total = opt_P_total;
OPTout.opt.P_batt  = opt_P_batt;
OPTout.opt.Q       = opt_Q;
OPTout.opt.I       = opt_I;
OPTout.opt.trq     = opt_Trq;
OPTout.opt.rpm     = opt_RPM;

% Baseline trajectory
OPTout.pred.v      = total_v;
OPTout.pred.acc    = total_a;
OPTout.pred.pos    = Physical_pos;
OPTout.pred.P      = pred_P;
OPTout.pred.I_batt = pred_I_batt;
OPTout.pred.trq    = pred_Trq;
OPTout.pred.rpm    = pred_RPM;

% Bounds + monitoring stats
OPTout.bound.v_max = up_buff_v;
OPTout.bound.v_min = do_buff_v;
OPTout.bound.a_max = up_buff_a;
OPTout.bound.a_min = do_buff_a;
OPTout.bound.vio_v = vio_v;
OPTout.bound.vio_a = vio_a;

end
