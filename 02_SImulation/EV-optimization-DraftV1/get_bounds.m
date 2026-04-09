function [up_buff_v, do_buff_v, up_buff_a, do_buff_a, total_v, total_a] = get_bounds(Pred_speds, param)
%GET_BOUNDS  1-step gap-based bounds (LOCAL, aligned with decision v_{k+1}).
%
% Pred_speds columns:
%   1  acc_val      [m/s^2]   (Vissim Acceleration)
%   3  spd^2        [(m/s)^2]
%   4  pos          [m]
%   5  dt           [s]
%   6  front gap    [m]
%   7  front speed  [m/s]
%   8  rear gap     [m]
%   9  rear speed   [m/s]
%
% OUTPUT (aligned):
%   up_buff_a/do_buff_a : (Np)x1 accel bounds for a1..aNp, where a_k=(v_{k+1}-v_k)/dt
%   up_buff_v/do_buff_v : (Np+1)x1 speed bounds for v1..v_{Np+1}
%                         NOTE: bounds for decision vars are v2..v_{Np+1} = (2:end)

    % ----------------------------
    % basics / guards
    % ----------------------------
    len = size(Pred_speds,1) - 1;
    Np  = min(param.Np, len);

    dist_tol = param.dist_tol;
    acc_tol  = param.acc_tol;

    dt = Pred_speds(1,5);
    if ~isfinite(dt) || dt <= 0
        dt = 0.5;
    end

    NO_CAR_DIST = 50;   % your convention (treat >= as "no car")

    % ----------------------------
    % baseline speed/acc (reference)
    % ----------------------------
    total_v = sqrt(max(Pred_speds(:,3),0));   % m/s, safe
    v1 = total_v(1);

    ds  = Pred_speds(2:len+1,4) - Pred_speds(1:len,4);
    dv2 = Pred_speds(2:len+1,3) - Pred_speds(1:len,3);

    total_a = zeros(len,1);
    nz = (ds ~= 0);
    total_a(nz) = dv2(nz) ./ (2*ds(nz));     % baseline accel surrogate

    % reference v_k for k=1..Np
    v_k = total_v(1:Np);

    % ----------------------------
    % read gaps & other speeds at step k=1..Np
    %   dF(k), vF(k), dR(k), vR(k) correspond to Pred_speds(k,:)
    % ----------------------------
    dF = Pred_speds(1:Np,6); dF = dF(:);
    dR = Pred_speds(1:Np,8); dR = dR(:);
    vF = Pred_speds(1:Np,7); vF = vF(:);
    vR = Pred_speds(1:Np,9); vR = vR(:);

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

    % =========================================================
    % 1-step safety constraints to bound NEXT speed v_{k+1}
    % (This matches your ABineq decision: x(k)=v_{k+1})
    %
    % Front:
    %   dF(k+1)=dF(k)+(vF(k)-vE(k+1))*dt >= dist_tol
    %   => vE(k+1) <= vF(k) + (dF(k)-dist_tol)/dt
    %
    % Rear:
    %   dR(k+1)=dR(k)+(vE(k+1)-vR(k))*dt >= dist_tol
    %   => vE(k+1) >= vR(k) + (dist_tol-dR(k))/dt
    % =========================================================
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
    % Convert NEXT-speed bounds -> accel bounds for a_k
    %   a_k = (v_{k+1} - v_k) / dt
    % =========================================================
    up_buff_a = (ub_v_next - v_k) ./ dt;
    do_buff_a = (lb_v_next - v_k) ./ dt;

    % relax accel bounds too if no vehicle
    up_buff_a(noFront) = acc_tol;
    do_buff_a(noRear)  = -acc_tol;

    % hard clamp accel
    up_buff_a(~isfinite(up_buff_a)) = acc_tol;
    do_buff_a(~isfinite(do_buff_a)) = -acc_tol;

    up_buff_a = min(max(up_buff_a, -acc_tol),  acc_tol);
    do_buff_a = min(max(do_buff_a, -acc_tol),  acc_tol);

    % ensure interval not inverted
    badA = up_buff_a < do_buff_a;
    if any(badA)
        mid = 0.5*(up_buff_a(badA) + do_buff_a(badA));
        up_buff_a(badA) = min(acc_tol, mid + 0.1);
        do_buff_a(badA) = max(-acc_tol, mid - 0.1);
    end

    % =========================================================
    % Build speed bounds v1..v_{Np+1}
    %   v_{k+1} = v_k + a_k*dt
    % So bounds for v2..v_{Np+1} are:
    %   v_{k+1} in [v_k + do_buff_a*dt, v_k + up_buff_a*dt]
    % =========================================================
    up_buff_v = [v1; v_k + up_buff_a*dt];
    do_buff_v = [v1; v_k + do_buff_a*dt];

    % nonnegative speeds (keep your convention)
    up_buff_v = max(up_buff_v, 0.2);
    do_buff_v = max(do_buff_v, 0.0);

    % final sanity
    badV = do_buff_v > up_buff_v;
    if any(badV)
        mid = 0.5*(do_buff_v(badV) + up_buff_v(badV));
        do_buff_v(badV) = max(0, mid - 0.1);
        up_buff_v(badV) = mid + 0.1;
    end
end
