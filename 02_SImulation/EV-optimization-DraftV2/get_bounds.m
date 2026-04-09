function [v_max_bound, v_min_bound, a_max_bound, a_min_bound, v_p , a_p] = get_bounds(Pred_speds, param)
%
% Pred_speds columns:
%   1  acc_val      [m/s^2]   (Vissim Acceleration)
%   3  spd^2        [(m/s)^2]
%   4  pos          [m]
%   5  dt           [s]
%   6  front gap    [m]
%   7  rear gap     [m]
%   8  rear gap speed     [m/s]
%   9  rear speed   [m/s]
%

    % ----------------------------
    % basics / guards
    % ----------------------------
    N = size(Pred_speds,1) - 1;
    Nx  = min(param.Np, N);

    dist_tol = param.dist_tol;
    acc_tol  = param.acc_tol;

    dt = Pred_speds(1,5);
    if ~isfinite(dt) || dt <= 0
        dt = 0.5;
    end

    NO_CAR_DIST = 50;   

    % ----------------------------
    % baseline speed/acc (reference)
    % ----------------------------
    v_p = sqrt(max(Pred_speds(:,3),0));   % m/s, safe
    v1 = v_p(1);

    ds  = Pred_speds(2:N+1,4) - Pred_speds(1:N,4);
    dv2 = Pred_speds(2:N+1,3) - Pred_speds(1:N,3);

    a_p = zeros(N,1);
    nz = (ds ~= 0);
    a_p(nz) = dv2(nz) ./ (2*ds(nz));     
    % reference v_k for k=1..Np
    v_k = v_p(1:Nx);

    % ----------------------------
    % read gaps & other speeds at step k=1..Np
    %   dF(k), vF(k), dR(k), vR(k) correspond to Pred_speds(k,:)
    % ----------------------------
    dF = Pred_speds(1:Nx,6); dF = dF(:);
    dR = Pred_speds(1:Nx,7); dR = dR(:);
    vF = Pred_speds(1:Nx,8); vF = vF(:);
    vR = Pred_speds(1:Nx,9); vR = vR(:);

    % clean
    dF(~isfinite(dF)) = NO_CAR_DIST;
    dR(~isfinite(dR)) = NO_CAR_DIST;
    vF(~isfinite(vF)) = 0;
    vR(~isfinite(vR)) = 0;

    dF = max(dF, 0);
    dR = max(dR, 0);
    vF = max(vF, 0);
    vR = max(vR, 0);

    noFront = (dF >= NO_CAR_DIST);
    noRear  = (dR >= NO_CAR_DIST);

    ub_v_next = vF + (dF - dist_tol) ./ dt;     % bound for v_{k+1}
    lb_v_next = vR + (dist_tol - dR) ./ dt;     % bound for v_{k+1}

    % relax when no vehicle
    ub_v_next(noFront) = inf;
    lb_v_next(noRear)  = 0;

    % nonnegative
    ub_v_next = max(ub_v_next, 0);
    lb_v_next = max(lb_v_next, 0);

    % fix local inversion (can happen when dF < dist_tol etc.)
    badGap = lb_v_next > ub_v_next;
    if any(badGap)
        mid = 0.5*(lb_v_next(badGap) + ub_v_next(badGap));
        lb_v_next(badGap) = max(0, mid - 0.5);
        ub_v_next(badGap) = mid + 0.5;
    end

    % =========================================================
    %   a_k = (v_{k+1} - v_k) / dt
    % =========================================================
    a_max_bound = (ub_v_next - v_k) ./ dt;
    a_min_bound = (lb_v_next - v_k) ./ dt;

    % relax accel bounds too if no vehicle
    a_max_bound(noFront) = acc_tol;
    a_min_bound(noRear)  = -acc_tol;

    % hard clamp accel
    a_max_bound(~isfinite(a_max_bound)) = acc_tol;
    a_min_bound(~isfinite(a_min_bound)) = -acc_tol;

    a_max_bound = min(max(a_max_bound, -acc_tol),  acc_tol);
    a_min_bound = min(max(a_min_bound, -acc_tol),  acc_tol);

    % ensure interval not inverted
    badA = a_max_bound < a_min_bound;
    if any(badA)
        mid = 0.5*(a_max_bound(badA) + a_min_bound(badA));
        a_max_bound(badA) = min(acc_tol, mid + 0.1);
        a_min_bound(badA) = max(-acc_tol, mid - 0.1);
    end

    v_max_bound = [v1; v_k + a_max_bound*dt];
    v_min_bound = [v1; v_k + a_min_bound*dt];

    v_max_bound = max(v_max_bound, 0.2);
    v_min_bound = max(v_min_bound, 0.0);

    badV = v_min_bound > v_max_bound;
    if any(badV)
        mid = 0.5*(v_min_bound(badV) + v_max_bound(badV));
        v_min_bound(badV) = max(0, mid - 0.1);
        v_max_bound(badV) = mid + 0.1;
    end
end
