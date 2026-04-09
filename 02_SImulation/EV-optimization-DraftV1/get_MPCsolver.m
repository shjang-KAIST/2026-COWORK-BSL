function MPCout = get_MPCsolver(param, Pred_speds, total_v, dist_ref, lb_v, ub_v, lb_a, ub_a, degmap, dist_init)
% GET_MPCSOLVER
% -------------------------------------------------------------------------
%   Pred_speds(:,5) : dt (step time)
%   total_v         : baseline speed rollout 
%   lb_v, ub_v      : speed bounds for v2..v_{Np+1}
%   lb_a, ub_a      : accel bounds for a1..aNp
%   dist_ref        : baseline distance reference
%
% Constraints:
%   1) speed bounds (lb_v <= v_dec <= ub_v)
%   2) accel bounds (lb_a <= (v(k+1)-v(k))/dt <= ub_a)
%   3) distance band: |dist - dist_ref| <= eps_dist
%   4) heat constraint: Q_gen <= Q_max (Joule surrogate)
%
% Cost:
%   - traction energy + thermal surrogate (P_0 + k_i*I^2)
% -------------------------------------------------------------------------

    Np = param.Np;

    %% [0] Shape / length guard
    dt = Pred_speds(1:Np,5); dt = dt(:);

    lb_v = lb_v(:); ub_v = ub_v(:);
    lb_a = lb_a(:); ub_a = ub_a(:);

    lb_v = lb_v(1:Np); ub_v = ub_v(1:Np);
    lb_a = lb_a(1:Np); ub_a = ub_a(1:Np);

    v0 = total_v(1);   % initial speed (fixed)

    %% [3] Initialization
    x0 = total_v(2:Np+1);
    x0 = x0(:);
    x0 = max(lb_v, min(ub_v, x0));

    [Aineq, bineq, dbg] = ABineq(Np, dt, lb_a, ub_a, v0, lb_v, ub_v, Pred_speds, param);

    %% [4] fmincon options 
    opts = optimoptions('fmincon', ...
        'Display','off', ...
        'Algorithm','sqp', ...
        'MaxFunctionEvaluations', 2e5, ...
        'MaxIterations', 400, ...
        'ConstraintTolerance', 1e-3, ...
        'StepTolerance', 1e-8, ...
        'OptimalityTolerance', 1e-4);

    %% [5] Problem definition
    % objective: Battery power + Heat 
    % nonlcon  : accel bounds + distance band + heat bound
    problem.objective = @(x) obj_fun(x, param, v0, Pred_speds, degmap, dist_init);
    problem.nonlcon   = @(x) nonlcon(x, param, v0, Pred_speds, degmap, dist_ref, dist_init);
    problem.x0        = x0;
    problem.solver    = 'fmincon';
    problem.options   = opts;
    problem.Aineq = Aineq;
    problem.bineq = bineq;

    %% [6] Solve
    [xopt, fval, exitflag, output] = fmincon(problem);

    % Fail-safe
    if exitflag <= 0
        warning('[MPC] Optimization failed to converge. (Exitflag: %d). Msg: %s', exitflag, output.message);
        xopt = x0;
    else
        fprintf('[MPC] Optimization SUCCESS | exitflag=%d | fval=%.4f | iters=%d\n', ...
        exitflag, fval, output.iterations);
    end

    %% [7] Pack outputs
    opt_v   = [v0; xopt(:)];
    opt_acc = (opt_v(2:end) - opt_v(1:end-1)) ./ dt;

    MPCout.opt_v    = opt_v;
    MPCout.opt_acc  = opt_acc;
    MPCout.exitflag = exitflag;
    MPCout.fval     = fval;

    % accel bounds
    MPCout.debug.lb_a_used = lb_a;
    MPCout.debug.ub_a_used = ub_a;
    
end


%% =========================================================================
% Objective: traction energy + thermal surrogate
% =========================================================================
function J = obj_fun(v_dec, param, v0, Pred, degmap, dist_init)
% Objective function:
%   - J_batt: traction energy (P_batt * dt)
%   - J_heat: cooling/thermal surrogate (P_0 + k_i*I^2)

    Np = param.Np;
    dt = Pred(1:Np,5); dt = dt(:);

    v_all = [v0; v_dec(:)];
    dist  = 0;
    J     = 0;

    for k = 1:Np
        acc  = (v_all(k+1) - v_all(k)) / dt(k);
        dist = dist + (v_all(k) + v_all(k+1))/2 * dt(k);

        EVout = get_evmodel(v_all(k), acc, dist, dist_init, param, degmap);

        % % thermal surrogate (cooling power)
        % P_cool = EVout.P_cool;
        % 
        % % traction power
        % P_batt = (EVout.P_batt);

        % stage cost
        J = J + EVout.P_total * dt(k);
    end
end


%% =========================================================================
% Nonlinear constraints:
% =========================================================================
function [cin, ceq] = nonlcon(v_dec, param, v0, Pred, degmap, dist_ref, dist_init)
% constraints:
%   - accel bounds: lb_a <= acc <= ub_a
%   - distance band: |dist - dist_ref| <= eps_dist
%   - heat bound: Q_gen(k) <= Q_max   (Joule surrogate: I^2*R_batt)

    Np = param.Np;
    dt = Pred(1:Np,5); dt = dt(:);

    v_all = [v0; v_dec(:)];
    acc   = (v_all(2:end) - v_all(1:end-1)) ./ dt;

    Q_gen   = zeros(Np,1);
    P_total = zeros(Np,1);


    dist_k = 0;
    for k = 1:Np
        dist_k =  dist_k  + (v_all(k) + v_all(k+1))/2 * dt(k);

        EVout = get_evmodel(v_all(k), acc(k), dist_k, dist_init, param, degmap);
        Q_gen(k)   = EVout.Q_batt;
        P_total(k) = EVout.P_total;
    end

    % power / heat
    c_power = P_total - param.P_max;      % <= 0
    c_heat  = Q_gen   - param.Q_max;      % <= 0

    % terminal distance band
    c_dist  = abs(dist_k - dist_ref) - param.eps_dist;   % <= 0

    cin = [c_power; c_heat;c_dist];
    ceq = [];
end


%% =========================================================================
% Linear constraints:
% =========================================================================
function [Aineq, bineq, dbg] = ABineq(Np, dt, lb_a, ub_a, v0, lb_v, ub_v, Pred_speds, param)
% ABineq (FEASIBILITY-SAFE)
% ------------------------------------------------------------
% Decision x = v_dec = [v2..v_{Np+1}] (Np x 1)
%
% Includes:
%  1) accel bounds (linear)
%  2) speed bounds (linear)
%  3) HARD 1-step gap-based speed bounds (linear via clamping lb_v/ub_v)
%
% Plus: post-clamp feasibility projection so (speed bounds) and (accel bounds)
%       don't contradict each other.
% ------------------------------------------------------------

    % --------- shape ----------
    dt   = dt(:);
    lb_a = lb_a(:);
    ub_a = ub_a(:);
    lb_v = lb_v(:);
    ub_v = ub_v(:);

    if numel(dt)~=Np || numel(lb_a)~=Np || numel(ub_a)~=Np || numel(lb_v)~=Np || numel(ub_v)~=Np
        error('ABineq: dt/lb_a/ub_a/lb_v/ub_v must have Np elements.');
    end

    % =========================================================
    % (0) Gap-based HARD clamp on speed bounds (per-step, local)
    % =========================================================
    dist_tol = param.dist_tol;
    NO_CAR_DIST = 50;

    dF = Pred_speds(1:Np,6); dF = dF(:);
    dR = Pred_speds(1:Np,8); dR = dR(:);
    vF = Pred_speds(1:Np,7); vF = vF(:);
    vR = Pred_speds(1:Np,9); vR = vR(:);

    % clean predictors
    dF(~isfinite(dF)) = NO_CAR_DIST;  dF = max(dF,0);
    dR(~isfinite(dR)) = NO_CAR_DIST;  dR = max(dR,0);
    vF(~isfinite(vF)) = 0;            vF = max(vF,0);
    vR(~isfinite(vR)) = 0;            vR = max(vR,0);

    hasFront = (dF < NO_CAR_DIST);
    hasRear  = (dR < NO_CAR_DIST);

    ub_gap = inf(Np,1);
    ub_gap(hasFront) = vF(hasFront) + (dF(hasFront) - dist_tol)./dt(hasFront);
    ub_gap = max(ub_gap, 0);

    lb_gap = zeros(Np,1);
    lb_gap(hasRear)  = vR(hasRear) + (dist_tol - dR(hasRear))./dt(hasRear);
    lb_gap = max(lb_gap, 0);

    % Clamp physical speed bounds with gap bounds
    ub_v0 = ub_v;  lb_v0 = lb_v;
    ub_v  = min(ub_v, ub_gap);
    lb_v  = max(lb_v, lb_gap);

    % (A) Fix inverted speed bounds after clamping
    bad = lb_v > ub_v;
    if any(bad)
        mid = 0.5*(lb_v(bad) + ub_v(bad));
        lb_v(bad) = max(0, mid - 0.1);
        ub_v(bad) = mid + 0.1;
    end

    % =========================================================
    % (1) Feasibility projection between accel bounds and speed bounds
    %     (prevents contradictions like: x1 >= v0+lb_a*dt but ub_v is smaller)
    % =========================================================
    % speed-implied accel feasibility (tighten accel bounds)
    % a_k = (v_{k+1}-v_k)/dt(k)
    % For k=1: v_k = v0, v_{k+1} = x1 in [lb_v(1), ub_v(1)]
    % For k>=2: v_k = x(k-1), v_{k+1} = x(k); with x bounded by lb_v/ub_v.
    %
    % Compute feasible accel interval given speed bounds only:
    a_min_possible = zeros(Np,1);
    a_max_possible = zeros(Np,1);

    % k=1 uses v0
    a_min_possible(1) = (lb_v(1) - v0) / dt(1);
    a_max_possible(1) = (ub_v(1) - v0) / dt(1);

    % k>=2 uses worst-case combinations to guarantee existence
    if Np >= 2
        for k = 2:Np
            % min accel occurs when next is minimal and current is maximal
            a_min_possible(k) = (lb_v(k)   - ub_v(k-1)) / dt(k);
            % max accel occurs when next is maximal and current is minimal
            a_max_possible(k) = (ub_v(k)   - lb_v(k-1)) / dt(k);
        end
    end

    % tighten accel bounds to those feasible ranges
    lb_a0 = lb_a; ub_a0 = ub_a;
    lb_a = max(lb_a, a_min_possible);
    ub_a = min(ub_a, a_max_possible);

    % fix inverted accel bounds if any (soften minimally)
    badA = lb_a > ub_a;
    if any(badA)
        mid = 0.5*(lb_a(badA) + ub_a(badA));
        lb_a(badA) = mid - 0.1;
        ub_a(badA) = mid + 0.1;
    end

    % =========================================================
    % (2) Allocate Aineq,bineq
    % =========================================================
    Aineq = zeros(4*Np, Np);
    bineq = zeros(4*Np, 1);

    % ----------------------------
    % (2-1) Accel constraints (2Np)
    % ----------------------------
    for k = 1:Np
        if k == 1
            % ub: (v2 - v0)/dt1 <= ub_a1  -> x1 <= ub_a1*dt1 + v0
            Aineq(k,1) = 1;
            bineq(k)   = ub_a(1)*dt(1) + v0;

            % lb: (v2 - v0)/dt1 >= lb_a1  -> -x1 <= -(lb_a1*dt1 + v0)
            Aineq(Np+k,1) = -1;
            bineq(Np+k)   = -(lb_a(1)*dt(1) + v0);
        else
            % ub: (xk - x(k-1))/dtk <= ub_a(k) -> xk - x(k-1) <= ub_a*dt
            Aineq(k,k)   =  1;
            Aineq(k,k-1) = -1;
            bineq(k)     =  ub_a(k)*dt(k);

            % lb: (xk - x(k-1))/dtk >= lb_a(k) -> x(k-1) - xk <= -lb_a*dt
            Aineq(Np+k,k)   = -1;
            Aineq(Np+k,k-1) =  1;
            bineq(Np+k)     = -lb_a(k)*dt(k);
        end
    end

    % ----------------------------
    % (2-2) Speed constraints (2Np)  (includes HARD gap clamp)
    % ----------------------------
    row0 = 2*Np;
    Aineq(row0 + (1:Np), :) = eye(Np);
    bineq(row0 + (1:Np))    = ub_v;

    row1 = 3*Np;
    Aineq(row1 + (1:Np), :) = -eye(Np);
    bineq(row1 + (1:Np))    = -lb_v;

    % =========================================================
    % Debug
    % =========================================================
    if nargout > 2
        dbg.ub_gap = ub_gap;
        dbg.lb_gap = lb_gap;
        dbg.ub_v_before = ub_v0;
        dbg.lb_v_before = lb_v0;
        dbg.ub_v_after  = ub_v;
        dbg.lb_v_after  = lb_v;

        dbg.lb_a_before = lb_a0;
        dbg.ub_a_before = ub_a0;
        dbg.lb_a_after  = lb_a;
        dbg.ub_a_after  = ub_a;

        dbg.a_min_possible = a_min_possible;
        dbg.a_max_possible = a_max_possible;
    end
end

