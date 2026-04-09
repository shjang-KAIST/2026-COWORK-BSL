function MPCout = get_MPCsolver(param, Pred_speds, total_v, dist_ref, ...
                                lb_v, ub_v, lb_a, ub_a, degmap, dist_init, batt0)
%GET_MPCSOLVER  Solve speed-profile MPC over horizon Np using fmincon (SQP)
% - Decision: x = v_dec = [v2 ... v_{Np+1}]  (Np x 1)
% - v0 = total_v(1) is fixed initial speed
% - Linear constraints: accel bounds + speed bounds (ABineq)
% - Nonlinear constraints: power/heat limits + distance matching
% - Objective: energy term (P_req*dt) + capacity-loss penalty (Q_loss*V_nom*Q_nom)

    Np = param.Np;

    %% (0) dt vector for horizon (length Np)
    dt = Pred_speds(1:Np,5);
    dt = dt(:);

    % ensure column + exact length Np
    lb_v = lb_v(:); ub_v = ub_v(:);
    lb_a = lb_a(:); ub_a = ub_a(:);

    lb_v = lb_v(1:Np); ub_v = ub_v(1:Np);
    lb_a = lb_a(1:Np); ub_a = ub_a(1:Np);

    % fixed initial speed
    v0 = total_v(1);

    %% (1) initial guess: baseline speeds clipped to bounds
    x0 = total_v(2:Np+1);
    x0 = max(lb_v, min(ub_v, x0));

    %% (2) Linear inequality constraints (accel + speed bounds)
    [Aineq, bineq] = ABineq(Np, dt, lb_a, ub_a, v0, lb_v, ub_v);

    %% (3) fmincon options (SQP)
    opts = optimoptions('fmincon', ...
        'Display','off', ...
        'Algorithm','sqp', ...
        'MaxFunctionEvaluations', 2e5, ...
        'MaxIterations', 400, ...
        'ConstraintTolerance', 1e-3, ...
        'StepTolerance', 1e-8, ...
        'OptimalityTolerance', 1e-4);

    %% (4) Problem setup
    problem.objective = @(x) obj_fun(x, param, v0, Pred_speds, degmap, dist_init, batt0);
    problem.nonlcon   = @(x) nonlcon(x, param, v0, Pred_speds, degmap, dist_ref, dist_init, batt0);
    problem.x0        = x0;
    problem.solver    = 'fmincon';
    problem.options   = opts;
    problem.Aineq     = Aineq;
    problem.bineq     = bineq;
    problem.lb        = lb_v;
    problem.ub        = ub_v;

    %% (5) Solve
    [xopt, fval, exitflag, output] = fmincon(problem);

    % fallback on failure
    if exitflag <= 0
        warning('[get_MPCsolver] Optimization failed. exitflag=%d', exitflag);
        xopt = x0;
    else
        fprintf('[get_MPCsolver] >> Optimization SUCCESS | exitflag=%d \n', exitflag);
    end

    %% (6) Pack results
    opt_v   = [v0; xopt(:)];                      % length Np+1
    opt_acc = (opt_v(2:end) - opt_v(1:end-1)) ./ dt; % length Np

    MPCout.opt_v    = opt_v;
    MPCout.opt_acc  = opt_acc;
    MPCout.exitflag = exitflag;
    MPCout.fval     = fval;
    MPCout.debug.output = output;
end


function J = obj_fun(v_dec, param, v0, Pred, degmap, dist_init, batt0)
%OBJ_FUN  Stage cost accumulation over horizon
% - Uses get_evmodel() to compute P_req and Q_loss per step
% - Cost: sum(P_req*dt) + sum(Q_loss*V_nom*Q_nom)

    Np = param.Np;
    dt = Pred(1:Np,5); dt = dt(:);

    v_all = [v0; v_dec(:)];

    dist = 0;
    J = 0;

    batt = batt0;

    for k = 1:Np
        v_k  = v_all(k);
        v_kp = v_all(k+1);

        acc  = (v_kp - v_k)/dt(k);
        dist = dist + v_k * dt(k);     % ZOH distance

        % step-wise dt override (variable dt supported)
        param_k = param;
        param_k.dt = dt(k);

        [EVout, batt] = get_evmodel(v_k, acc, dist, dist_init, param_k, degmap, batt);

        V_nom = param.V_nom;
        Q_nom = param.Q_max_Ah;

        alpha = 0.8;
        J = J + alpha * EVout.P_req * dt(k) + (1-alpha) * EVout.Q_loss * V_nom * Q_nom;
    end
end


function [cin, ceq] = nonlcon(v_dec, param, v0, Pred, degmap, dist_ref, dist_init, batt0)
%NONLCON  Nonlinear inequality constraints
% - power limit: P_req <= P_max
% - heat limit : Q_batt <= Q_max
% - distance matching: |dist_end - dist_ref| <= eps_dist

    Np = param.Np;
    dt = Pred(1:Np,5); dt = dt(:);

    v_all = [v0; v_dec(:)];

    batt = batt0;

    dist  = 0;
    P_req = zeros(Np,1);
    Q_gen = zeros(Np,1);

    for k = 1:Np
        v_k  = v_all(k);
        v_kp = v_all(k+1);

        acc  = (v_kp - v_k)/dt(k);
        dist = dist + v_k * dt(k);

        param_k = param;
        param_k.dt = dt(k);

        [EVout, batt] = get_evmodel(v_k, acc, dist, dist_init, param_k, degmap, batt);

        P_req(k) = EVout.P_req;
        Q_gen(k) = EVout.Q_batt;
    end

    c_power = P_req - param.P_max;
    c_heat  = Q_gen - param.Q_max;
    c_dist  = abs(dist - dist_ref) - param.eps_dist;

    cin = [c_power; c_heat; c_dist];
    ceq = [];
end



function [Aineq, bineq] = ABineq(Np, dt, lb_a, ub_a, v0, lb_v, ub_v)
%ABINEQ  Linear inequality constraints for fmincon
% Decision x = [v2..v_{Np+1}] (Np x 1)
% - accel bounds:
%     lb_a(k) <= (v_{k+1}-v_k)/dt(k) <= ub_a(k)
% - speed bounds:
%     lb_v(k) <= v_{k+1} <= ub_v(k)

    Aineq = zeros(4*Np, Np);
    bineq = zeros(4*Np, 1);

    % accel constraints (uses v0 for k=1)
    for k = 1:Np
        if k == 1
            Aineq(k,1) = 1;
            bineq(k) = ub_a(1)*dt(1) + v0;

            Aineq(Np+k,1) = -1;
            bineq(Np+k) = -(lb_a(1)*dt(1) + v0);
        else
            Aineq(k,k)   =  1;
            Aineq(k,k-1) = -1;
            bineq(k)     =  ub_a(k)*dt(k);

            Aineq(Np+k,k)   = -1;
            Aineq(Np+k,k-1) =  1;
            bineq(Np+k)     = -lb_a(k)*dt(k);
        end
    end

    % speed bounds for decision variables (v2..v_{Np+1})
    Aineq(2*Np + (1:Np), :) = eye(Np);
    bineq(2*Np + (1:Np))    = ub_v;

    Aineq(3*Np + (1:Np), :) = -eye(Np);
    bineq(3*Np + (1:Np))    = -lb_v;
end
