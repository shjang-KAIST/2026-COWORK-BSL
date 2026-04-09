function [EVout, batt_next] = get_evmodel(v_curr, acc, dist_m, dist_init, param, degmap, batt)
% GET_EVMODEL
% EV traction + battery current + pack thermal dynamics + cooling control + aging
%
% Consistent hierarchy:
%   Cell  -> Module -> Pack
%   Module: 16s24p
%   Pack  : 6 modules in series  => 96s24p
%
% Modeling assumptions:
%   - Capacity fade / resistance increment: cell level
%   - OCV polynomial: module level
%   - Current, power, thermal states: pack level
%
% Required fields (example):
%   batt.Q_bat      : current pack capacity state [Ah]
%   batt.T          : pack temperature [K]
%   batt.Ah_th      : accumulated cell Ah-throughput [Ah]
%   batt.I_prev     : previous pack current [A] (optional)
%
%   param.Np        : number of parallel cells per module (=24)
%   param.Ns_mod    : number of series-connected cells per module (=16)
%   param.N_mod_s   : number of modules in series (=6)
%   param.Q_bat0    : initial pack capacity [Ah]
%   param.Q_bat_nom : nominal pack capacity [Ah]
%   param.Fit_IR    : module internal resistance fit object
%   param.OCV_sys   : module OCV polynomial coefficients
%   param.OCV_mu    : normalization parameters for module OCV input
%
% Note:
%   This function updates usable charge Q_bat by current integration.
%   Capacity fade is exported as cumulative Q_loss [%] following an
%   Onori-style Ah-throughput aging formulation. Pack resistance follows
%   the fitted module IR map and is scaled to string level.

    %% 0) basic settings
    dtiny = 1e-9;
    dt    = param.dt;

    %% 1) vehicle speed
    v = max(0, v_curr);   % [m/s]

    %% 2) road slope
    grade      = get_slope(dist_m, degmap, dist_init);
    theta_road = atan(grade);

    %% 3) longitudinal torque demand
    mass_eq = param.mass * param.mass_factor;

    Trq_acc  = mass_eq * acc * param.R_tire / param.gear_ratio;
    Trq_air  = param.k_air * (v + param.wind_sped)^2 * param.R_tire / param.gear_ratio;
    Trq_grad = param.mass * param.gravity * sin(theta_road) * param.R_tire / param.gear_ratio;
    Trq_roll = param.RRC * param.mass * cos(theta_road) * param.gravity * param.R_tire / param.gear_ratio;

    T_mech = Trq_acc + Trq_air + Trq_grad + Trq_roll;
    wm     = abs(v / param.R_tire) * param.gear_ratio;   % [rad/s]
    rpm    = wm * 30 / pi;

    %% 4) traction electrical power
    v_kmh = v * 3.6;
    T_mot = usr_MOVE2MT(acc, v_kmh);

    P_raw = 1e3 * usr_MT2BC_sys(T_mot, v_kmh, param.BPsys);   % [W]
    if P_raw >= 0
        P_trac = P_raw;
    else
        P_trac = usr_REGENBP(acc, P_raw);   % keep <= 0 in regen
    end

    %% 5) battery configuration
    Np      = param.Np;       % parallel cells per module
    Ns_mod  = param.Ns_mod;   % series cells per module
    N_mod_s = param.N_mod_s;  % series modules per pack

    N_series_total = Ns_mod * N_mod_s;   % total series cell count

    %% 6) battery states and capacity hierarchy
    % Pack-level capacity state [Ah]
    if isfield(batt, 'Q_bat') && ~isempty(batt.Q_bat)
        Q_bat = batt.Q_bat;
    elseif isfield(batt, 'Q_Ah') && ~isempty(batt.Q_Ah)
        Q_bat = batt.Q_Ah;
    else
        Q_bat = param.Q_bat0;
    end
    Q_bat0  = param.Q_bat0;
    Q_bat_nom = param.Q_bat_nom;

    % Since parallel branches add Ah:
    % Q_bat = Np * Q_cell
    Q_cell  = Q_bat  / Np;
    Q_cell0 = Q_bat0 / Np;

    % Module capacity equals pack capacity in pure series module stacking
    Q_mod = Q_bat;   % [Ah]

    % SOC: same whether computed at pack/module/cell for pure series stacking
    SOC = Q_bat / Q_bat_nom;
    SOC = min(max(SOC, 0), 1);

    %% 7) fitted module-level resistance -> string resistance
    if isfield(batt, 'Ah_ir') && ~isempty(batt.Ah_ir)
        Ah_ir = batt.Ah_ir;
    else
        Ah_ir = 0;
    end

    if isfield(batt, 'T') && ~isempty(batt.T)
        T_pack = batt.T;
    else
        T_pack = param.T_init;
    end

    T_mod_C = T_pack - 273.15;
    Ah_mod = max(Ah_ir, param.ir_ah_floor);

    R_mod = feval(param.Fit_IR, T_mod_C, Ah_mod);
    if ~isfinite(R_mod) || R_mod <= 0
        R_mod = param.R_mod0;
    end
    R_int_pack = max(dtiny, R_mod * N_mod_s);   % [ohm], string/pack level

    %% 9) module-level OCV model -> pack OCV
    p  = param.OCV_sys(:);
    mu = param.OCV_mu(:);

    % OCV polynomial input is module capacity (or normalized module quantity)
    x = (Q_mod - mu(1)) / max(mu(2), dtiny);

    OCV_mod = p(1)*x.^6 + p(2)*x.^5 + p(3)*x.^4 + ...
              p(4)*x.^3 + p(5)*x.^2 + p(6)*x + p(7);

    OCV_pack = OCV_mod * N_mod_s;

    if ~isfinite(OCV_pack) || OCV_pack <= 0
        OCV_pack = max(1.0, abs(OCV_pack));
    end

    %% 10) BTMS control based on current pack temperature
    P_cool = 0;
    P_heat = 0;

    eT = T_pack - param.T_ref;

    if eT > param.T_deadband
        P_cool = param.P_0 + param.Kp_T * (eT - param.T_deadband);
    elseif eT < -param.T_deadband
        P_heat = param.Pheat_0 + param.Kp_heat * (-eT - param.T_deadband);
    end

    P_cool = min(max(P_cool, 0), param.Pcool_max);
    P_heat = min(max(P_heat, 0), param.Pheat_max);
    P_tms  = P_cool + P_heat;
    T_ref = param.T_ref;

    %% 11) battery current solve including BTMS load
    P_req = P_trac + P_tms;

    % Pack power relation:
    %   P_req = OCV_pack * I_batt - R_int_pack * I_batt^2
    disc = OCV_pack^2 - 4 * R_int_pack * P_req;
    disc = max(0, disc);

    I_batt = (OCV_pack - sqrt(disc)) / (2 * max(R_int_pack, dtiny));   % [A], pack current
    I_cell = I_batt / Np;                                               % [A], cell current

    %% 12) joule heating and thermal dynamics
    Q_joule = I_batt^2 * R_int_pack;               % [W]
    Q_env   = param.hA * (T_pack - param.T_amb);   % [W]
    Q_cool  = param.eta_cool * P_cool;             % [W_th]
    Q_heat  = param.COP_heat * P_heat;             % [W_th]

    Cth = param.Cth;   % [J/K]
    T_next = T_pack + dt / Cth * (Q_joule - Q_env - Q_cool + Q_heat);

    %% 13) charge update
    Q_bat_next = Q_bat - I_batt * (dt / 3600);   % [Ah]
    Q_bat_next = min(max(Q_bat_next, 0), Q_bat_nom);
    Ah_ir_next = Ah_ir + abs(I_batt) * (dt / 3600);

    SOC_next = Q_bat_next / max(Q_bat_nom, dtiny);
    SOC_next = min(max(SOC_next, 0), 1);

    %% 14) aging model
    % Cell-level Ah throughput accumulation
    if isfield(batt, 'Ah_th') && ~isempty(batt.Ah_th)
        Ah_th = batt.Ah_th;
    else
        Ah_th = 0;
    end
    Ah_th_next = Ah_th + abs(I_cell) * (dt / 3600);

    if isfield(batt, 'Ah_eff') && ~isempty(batt.Ah_eff)
        Ah_eff = batt.Ah_eff;
    else
        Ah_eff = 0;
    end

    if SOC_next <= 0.45
        alpha_age = 1287.6;
        beta_age  = 6356.3;
    else
        alpha_age = 1385.5;
        beta_age  = 4193.2;
    end

    % Cell-level C-rate
    Q_cell_nom = Q_bat_nom / Np;          % [Ah]
    I_c        = abs(I_cell) / max(Q_cell_nom, dtiny);   % [1/h]
    theta      = T_next;                  % [K]
    R_gas      = param.R_gas;
    age_exp    = param.age_exp;

    age_factor = (alpha_age * SOC_next + beta_age) * ...
                 exp((-31700 + 163.3 * I_c) / (R_gas * theta));
    Q_loss = age_factor * Ah_th_next^age_exp;   % cumulative capacity loss [%]

    soc_ref = min(max(param.age_soc_ref, 0), 1);
    if soc_ref <= 0.45
        alpha_ref = 1287.6;
        beta_ref  = 6356.3;
    else
        alpha_ref = 1385.5;
        beta_ref  = 4193.2;
    end

    age_factor_ref = (alpha_ref * soc_ref + beta_ref) * ...
                     exp((-31700 + 163.3 * param.age_crate_ref) / ...
                     (R_gas * param.age_temp_ref));

    gamma = (param.age_qref_pct / max(age_factor, dtiny))^(1 / age_exp);
    Gamma = (param.age_qref_pct / max(age_factor_ref, dtiny))^(1 / age_exp);
    sigma = Gamma / max(gamma, dtiny);

    Aheff_next = Ah_eff + sigma * abs(I_cell) * (dt / 3600);

    %% 15) next battery state
    batt_next         = batt;
    batt_next.Q_bat   = Q_bat_next;   % pack capacity state [Ah]
    batt_next.Q_Ah    = Q_bat_next;   % backward-compatible alias [Ah]
    batt_next.I_prev  = I_batt;       % pack current [A]
    batt_next.Rb      = R_int_pack;   % pack resistance [ohm]
    batt_next.R_str0  = R_int_pack;   % backward-compatible alias [ohm]
    batt_next.R_mod   = R_mod;        % module resistance [ohm]
    batt_next.Ah_ir   = Ah_ir_next;   % module/pack throughput for IR fit [Ah]
    batt_next.Ah_th   = Ah_th_next;   % cell Ah-throughput [Ah]
    batt_next.Ah_eff  = Aheff_next;   % effective cell Ah-throughput [Ah]
    batt_next.T       = T_next;       % pack temperature [K]

    %% 16) outputs
    EVout.P_req    = P_req;
    EVout.P_trac   = P_trac;
    EVout.P_cool   = P_cool;
    EVout.P_heat   = P_heat;
    EVout.P_tms    = P_tms;

    EVout.Q_batt   = Q_joule;   % joule heat [W]
    EVout.Q_env    = Q_env;
    EVout.Q_cool   = Q_cool;
    EVout.Q_heat   = Q_heat;

    EVout.I_batt   = I_batt;
    EVout.I_cell   = I_cell;

    EVout.T_batt   = T_next;
    EVout.T_ref    = T_ref;

    EVout.Q_bat    = Q_bat_next;
    EVout.Q_Ah     = Q_bat_next;
    EVout.SOC      = 100 * SOC_next;

    EVout.OCV_mod  = OCV_mod;
    EVout.OCV      = OCV_pack;
    EVout.trq      = T_mech;
    EVout.tmot     = T_mot;
    EVout.rpm      = rpm;
    EVout.grade    = grade;
    EVout.Ah_ir    = Ah_ir_next;
    EVout.R_mod    = R_mod;
    EVout.Rin      = R_int_pack;
    EVout.Q_loss   = Q_loss;
    EVout.gamma    = gamma;
    EVout.Gamma    = Gamma;
    EVout.sigma    = sigma;
    EVout.Ah_eff   = Aheff_next;
    EVout.AgeCost  = sigma * abs(I_cell);

end
