function [EVout, batt_next] = get_evmodel(v_curr, acc, dist_m, dist_init, param, degmap, batt)
%GET_EVMODEL  EV traction + battery current + cooling surrogate + heat + aging metrics
%
    v = max(0, v_curr);
    dtiny = 1e-9;

    dt = param.dt;

    %% 1) Road slope
    grade = get_slope(dist_m, degmap, dist_init);   % grade [-], dimensionless
    theta = atan(grade);                            % road angle [rad]

    %% 2) Longitudinal torque demand 
    mass_eq = param.mass * param.mass_factor;

    Trq_acc  = mass_eq * acc * param.R / param.gear_ratio;
    Trq_air  = param.k_air * (v + param.wind_sped)^2 * param.R / param.gear_ratio;
    Trq_grad = param.mass * param.gravity * sin(theta) * param.R / param.gear_ratio;
    Trq_roll = param.RRC * param.mass * cos(theta) * param.gravity * param.R / param.gear_ratio;

    T_mech = Trq_acc + Trq_air + Trq_grad + Trq_roll;
    rpm    = abs(v / max(dtiny,param.R) * param.gear_ratio * 30 / pi);

    %% 3) Traction electrical power from BP model
    % P_trac: electrical power demand at DC bus [W]
    % (+) motoring, (-) regen
    v_kmh = v * 3.6;
    T_mot = usr_MOVE2MT(acc, v_kmh);

    P_raw = 1e3 * usr_MT2BC_sys(T_mot, v_kmh, param.BPsys);  % [W]
    if P_raw >= 0
        P_trac = P_raw;
    else
        P_trac = usr_REGENBP(acc, P_raw); % keep <=0 in regen
    end

    %% 4) Battery electrical model (string/pack mapping) + OCV estimate
    % Nser_pack : number of modules in series per string (pack voltage scales with this)
    % Npar_pack : number of parallel strings in the pack (pack current splits across this)
    Nser_pack = param.Nser;               % modules in series in ONE string
    Npar_pack = max(1,param.Npar);        % number of strings in parallel (pack-level)

    % ---- capacity state (string-based) ----
    % Q_str is the charge state of ONE string [Ah]
    % (pack total capacity would be Npar_pack * Q_str)
    Q_str  = batt.Q_Ah;                   % [Ah] per string
    Q_str0 = param.Q_init0;               % [Ah] per string (BOL)

    % ---- capacity fade ----
    % C_dec_str: charge depletion from BOL in [Ah], per string
    C_dec_str = max(0, Q_str0 - Q_str);

    % ---- resistance increment model  ----
    % Paper: A new state-of-health estimation method for lithium-ion batteries through the intrinsic relationship between ohmic internal resistance and capacity
    a = 13.565;  b = 0.0073;
    R_inc_mod = max(0, (C_dec_str - b)/a);

    % ---- base resistance ----
    % param.R_str0 is treated as BASE module terminal resistance [ohm]
    R_str0 = param.R_str0;                % [ohm] base (per module terminal)
    R_str  = R_str0 + R_inc_mod;          % [ohm] updated (per module terminal)

    % ---- pack resistance mapping ----
    % String resistance: Nser_pack series modules
    % Pack equivalent: parallel strings reduce resistance by Npar_pack
    R_str  = Nser_pack * R_str;           % [ohm] string resistance
    R_pack = R_str / Npar_pack;           % [ohm] pack equivalent resistance

    % ---- OCV model ----
    % OCV polynomial is evaluated in Ah-domain of the STRING state (Q_str),
    % then scaled by Nser_pack to obtain pack OCV
    p  = param.OCV_sys(:);
    mu = param.OCV_mu(:);
    x  = (Q_str - mu(1)) / max(dtiny, mu(2));

    OCV = (p(1)*x.^6 + p(2)*x.^5 + p(3)*x.^4 + p(4)*x.^3 + p(5)*x.^2 + p(6)*x + p(7)) * Nser_pack;
    if ~isfinite(OCV) || OCV <= 0
        OCV = max(1.0, abs(OCV));
    end

    %% 5) Battery pack current with cooling load (fixed-point method)
    % Solve battery quadratic:  P_req = OCV*I - R_pack*I^2
    % Cooling power depends on Joule loss R_pack*I^2 through COP model.
    % Paper: Two-Layer Model Predictive Battery Thermal and Energy Management Optimization for Connected and Automated Electric Vehicles (CDC)
    P0  = param.P_0;
    ki  = param.k_i;
    COP = param.COP;

    % Aging model constants (paper):
    % R     : universal gas constant [J/(mol·K)] (8.314)
    % theta : battery temperature [K]
    % Q_nom : nominal capacity [Ah] (string basis)
    R     = param.R;
    theta = param.theta;
    Q_nom = param.Q_max_Ah;

    I_batt = batt.I_prev;                       % [A] pack terminal current
    if ~isfinite(I_batt), I_batt = 0; end

    itMax = 100;
    tolI  = 1e-3;

    for k = 1:itMax
        P_cool = P0 + ki * (R_pack * I_batt^2) / COP;       % [W]
        P_req  = P_trac + P_cool;                           % [W]

        disc = OCV^2 - 4*R_pack*P_req;                      % discriminant
        disc = max(0, disc);

        I_new = (OCV - sqrt(disc)) / (2*max(R_pack,1e-9));  % [A] pack current

        if abs(I_new - I_batt) < tolI
            I_batt = I_new;
            break;
        end
        I_batt = I_new;
    end

    % Current split across parallel strings 
    I_str = I_batt / Npar_pack;           % [A] per string

    % finalize powers with converged I_batt
    P_cool = P0 + ki * (R_pack * I_batt^2) / max(dtiny, COP);
    P_req  = P_trac + P_cool;

    % Joule heat at pack terminals
    Q_batt = (I_batt^2) * R_pack;         % [W] (electrical loss)

    % ---- string SOC update  ----
    % Q_str integrates per-string current I_str
    Q_next = Q_str - I_str * (dt/3600.0); % [Ah] per string
    Q_next = min(max(Q_next, 0), Q_nom);

    % ---- Ah-throughput update (paper's Ah term) ----
    % Ah_th is cumulative integral of |I_str| over time, per string
    Ah_th_next = batt.Ah_th + abs(I_str) * (dt/3600);

    % SOC (Nomorlized)
    SOC = Q_next / max(dtiny, param.Q_max_Ah);

    %% Battery-degradation model-Paper: Energy Management Strategy for HEVs Including Battery Life Optimization
    if SOC <= 0.45
        alpha = 1287.6; beta = 6356.3;
    else
        alpha = 1385.5; beta = 4193.2;
    end

    % C-rate definition (per string): I_c = |I_str| / Q_nom   [1/h]
    I_c = abs(I_str) / Q_nom;

    % Capacity loss model
    % Q_loss (%)
    Q_loss = (alpha * SOC + beta) * exp((-31700 + 163.3 * I_c)/ (R * theta)) * Ah_th_next^(0.57);

    %% pack outputs / state update
    batt_next = batt;
    batt_next.Q_Ah   = Q_next;            % [Ah] per string
    batt_next.I_prev = I_batt;            % [A] pack
    batt_next.Rb     = R_pack;            % [ohm] pack equivalent
    batt_next.Ah_th  = Ah_th_next;        % [Ah] per string throughput

    %% 6) Output struct (reporting)
    EVout.P_req   = P_req;
    EVout.P_trac  = P_trac;
    EVout.P_cool  = P_cool;
    EVout.Q_batt  = Q_batt;
    EVout.I_batt  = I_batt;               % [A] pack

    EVout.Q_Ah    = Q_next;               % [Ah] per string
    EVout.SOC     = 100 * SOC;            % [%] reported (internally SOC is 0..1)

    EVout.OCV     = OCV;                  % [V] pack OCV
    EVout.trq     = T_mech;
    EVout.tmot    = T_mot;
    EVout.rpm     = rpm;
    EVout.grade   = grade;
    EVout.Rin     = R_pack;               % [ohm] pack
    EVout.Q_loss  = Q_loss;               % [%] capacity loss metric 
end
