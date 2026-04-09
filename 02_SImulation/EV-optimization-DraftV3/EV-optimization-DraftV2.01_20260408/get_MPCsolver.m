function MPCout = get_MPCsolver(param, Pred_speds, total_v, dist_ref, ...
                                lb_v, ub_v, lb_a, ub_a, degmap, dist_init, batt0, alpha)
%GET_MPCSOLVER  Solve speed-profile MPC over horizon Np using fmincon (SQP)
% - Decision: x = v_dec = [v2 ... v_{Np+1}]  (Np x 1)
% - v0 = total_v(1) is fixed initial speed
% - Linear constraints: accel bounds + speed bounds (ABineq)
% - Nonlinear constraints: power/heat limits + distance matching
% - Objective: normalized traction/BTMS energy + normalized aging term

    Np = param.N_pred;

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
    problem.objective = @(x) obj_fun(x, param, v0, Pred_speds, degmap, dist_init, batt0, alpha);
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

function J = obj_fun(v_dec, param, v0, Pred, degmap, dist_init, batt0, alpha)
%OBJ_FUN  Stage cost accumulation over horizon
% - Uses get_evmodel() to compute traction/BTMS power and aging severity
% - Cost: alpha * normalized energy + (1-alpha) * normalized aging

    Np = param.N_pred;
    dt = Pred(1:Np,5); dt = dt(:);

    v_all = [v0; v_dec(:)];

    dist = 0;
    J = 0;

    batt = batt0;
    dtiny = 1e-9;
    E_ref = max(param.V_nom * param.Q_bat_nom, dtiny); % [Wh]
    I_cell_ref = max(param.P_max / max(param.V_nom, dtiny) / param.Np, dtiny); % [A]

    for k = 1:Np
        v_k  = v_all(k);
        v_kp = v_all(k+1);

        acc  = (v_kp - v_k)/dt(k);
        dist = dist + v_k * dt(k);     % ZOH distance

        % step-wise dt override (variable dt supported)
        param_k = param;
        param_k.dt = dt(k);

        [EVout, batt] = get_evmodel(v_k, acc, dist, dist_init, param_k, degmap, batt);
        E_trac_Wh = EVout.P_trac * dt(k) / 3600;
        E_tms_Wh  = EVout.P_tms  * dt(k) / 3600;
        J_energy  = (param.obj_w_trac * E_trac_Wh + ...
                     param.obj_w_tms  * E_tms_Wh) / E_ref;

        % Onori-style instantaneous battery aging cost:
        % sigma(SOC, Ic, T) * |I_cell|
        J_deg = EVout.AgeCost / I_cell_ref;

        J = J + alpha * J_energy + (1 - alpha) * J_deg;
        
    end
end


function [cin, ceq] = nonlcon(v_dec, param, v0, Pred, degmap, dist_ref, dist_init, batt0)
%NONLCON  Nonlinear inequality constraints
% - power limit: P_req <= P_max
% - heat limit : Q_batt <= Q_max
% - BTMS power limit : P_tms <= Ptms_max
% - temperature limit : T_batt_min <= T_batt <= T_batt_max
% - distance matching: |dist_end - dist_ref| <= eps_dist

    Np = param.N_pred;
    dt = Pred(1:Np,5); dt = dt(:);

    v_all = [v0; v_dec(:)];

    batt = batt0;

    dist  = 0;
    P_req = zeros(Np,1);
    Q_gen  = zeros(Np,1);
    P_tms  = zeros(Np,1);
    T_batt = zeros(Np,1);

    for k = 1:Np
        v_k  = v_all(k);
        v_kp = v_all(k+1);

        acc  = (v_kp - v_k)/dt(k);
        dist = dist + v_k * dt(k);

        param_k = param;
        param_k.dt = dt(k);

        [EVout, batt] = get_evmodel(v_k, acc, dist, dist_init, param_k, degmap, batt);

        P_req(k)  = EVout.P_req;
        Q_gen(k)  = EVout.Q_batt;
        P_tms(k)  = EVout.P_tms;
        T_batt(k) = EVout.T_batt;
    end

    c_power = P_req - param.P_max;
    c_heat  = Q_gen - param.Q_max;
    c_tms   = P_tms - param.Ptms_max;
    c_temp_hi = T_batt - param.T_batt_max;
    c_temp_lo = param.T_batt_min - T_batt;
    c_dist  = abs(dist - dist_ref) - param.eps_dist;

    cin = [c_power; c_heat; c_tms; c_temp_hi; c_temp_lo; c_dist];
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

