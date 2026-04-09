function EVout = get_evmodel(v_curr, acc, dist_m, dist_init, param, degmap)
%GET_EVMODEL  EV traction + thermal + cooling + current (struct output)
%
% Inputs
%   v_curr  [m/s]
%   acc     [m/s^2]
%   dist_m  [m]
%   param   struct (must contain fields below)
%   degmap  [N x 2]  (col1: distance [m], col2: slope "grade" = tan(theta) OR percent/deg depending on your map)
%
% Required param fields
%   mass, mass_factor, R, gear_ratio, k_air, wind_sped, gravity, RRC
%   use_bp_model (true/false)
%   (if use_bp_model) BP_sys, regen_eff (or usr_REGENBP handles internally)
%   V_nom [V]
%   R_batt_eq [ohm]
%   P_cool_base [W], cool_coeff [-]  (P_cool = P_cool_base + cool_coeff*Q_heat)
%
% Outputs (EVout)
%   P_total, P_batt, trq, rpm, P_cool, Q_heat, I_batt
%
% Notes
% - grade_map is treated as "grade = tan(theta)" by default.
% - If your degmap(:,2) is DEGREE, change the conversion below (see "MAP TYPE" section).

    %% 0) sanitize
    v = max(0, v_curr);  
    dtiny = 1e-9;

    %% 1) Road slope (MAP TYPE)
    grade = get_slope(dist_m, degmap, dist_init);     % dimensionless
    theta_rad = atan(grade);              % rad

    %% 2) Longitudinal torque demand (at motor side)
    mass_eq = param.mass * param.mass_factor;

    Trq_acc  = mass_eq * acc * param.R / param.gear_ratio;
    Trq_air  = param.k_air * (v + param.wind_sped)^2 * param.R / param.gear_ratio;
    Trq_grad = param.mass * param.gravity * sin(theta_rad) * param.R / param.gear_ratio;
    Trq_roll = param.RRC * param.mass * cos(theta_rad) * param.gravity * param.R / param.gear_ratio;

    T_mech = Trq_acc + Trq_air + Trq_grad + Trq_roll;

    rpm   = abs(v / param.R * param.gear_ratio * 30 / pi);
    omega = (v / param.R * param.gear_ratio);  % [rad/s] 

    %% 3) Traction power (Battery power) from BP model 
        v_kmh = v * 3.6;
        T_mot = usr_MOVE2MT(acc, v_kmh);
        P_raw = 1e3 * usr_MT2BC_sys(T_mot, v_kmh, param.BPsys);  % [W], +drive, -regen (assumed)

        if P_raw >= 0
            % Traction power
            P_batt = P_raw;
        else
            % Regenerative power
            P_batt = usr_REGENBP(acc, P_raw);
        end


    %% 4) Thermal model (I^2 R + drivetrain loss) + cooling power

    % ============================================================
    % Battery current closure: P_cool = P_0 + k_i * I^2 (same battery)
    % Solve: k_i I^2 - Vnom I + (P_batt + P_0) = 0
    % ============================================================
    
    Vnom  = max(dtiny, param.V_nom);
    
    P0 = param.P_0;
    ki = param.k_i;
    
    % (1) default: if ki ~ 0 => linear
    if abs(ki) < 1e-12
        % P_cool = P0, P_total = P_batt + P0
        P_cool  = P0;
        P_total = P_batt + P_cool;
        I_batt  = P_total / Vnom;
    
    else
        % (2) quadratic discriminant
        D = Vnom^2 - 4*ki*(P_batt + P0);
    
        if D >= 0
            sqrtD = sqrt(D);
    
            % I1 = (Vnom - sqrtD)/(2ki)  
            % I2 = (Vnom + sqrtD)/(2ki)  
            I1 = (Vnom - sqrtD) / (2*ki);
            I2 = (Vnom + sqrtD) / (2*ki);
    
            % pick the root that is closer to "no-cooling" current (continuity)
            I_ref = (P_batt / Vnom);
            if abs(I1 - I_ref) <= abs(I2 - I_ref)
                I_batt = I1;
            else
                I_batt = I2;
            end
    
            % now compute powers consistently
            P_cool  = P0 + ki * I_batt^2;
            P_total = P_batt + P_cool;
    
        else
            % D < 0 => no real solution under this surrogate.
            sqrtD  = 0;
            I_batt = (Vnom - sqrtD) / (2*ki);   % = Vnom/(2ki)
    
            P_cool  = P0 + ki * I_batt^2;
            P_total = P_batt + P_cool;
        end
    end
    
    % ohmic heat (based on final I_batt)
    Q_batt = (I_batt^2) * max(dtiny, param.R_batt);

    %% 5) Output
    EVout.P_total = P_total;
    EVout.P_batt  = P_batt;
    EVout.trq     = T_mech;
    EVout.rpm     = rpm;
    EVout.P_cool  = P_cool;
    EVout.Q_batt  = Q_batt;
    EVout.I_batt  = I_batt;
end
