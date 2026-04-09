function param = get_parameters(input)
    % GET_CONFIG: 
    % Vissim setup parameters
    param.total_flow = input;    % Total Flow [veh/hr]
    param.flow_rate1 = 0.7;     % Flow Rate Section 1
    param.flow_rate2 = 0.06;    % Flow Rate Section 2
    param.flow_rate3 = 0.03;    % Flow Rate Section 3
    param.flow_rate4 = 0.21;    % Flow Rate Section 4
    
    param.random_seed = 42;     % Random Seed for Simulation

    
    param.dt = 0.2;             % Time Step [s]
    param.sim_speed_10 = 10;       % Simulation Speed (default)

    param.target_pos = 2000;

    param.warmup_step = round(400 / param.dt);  % Warm-up Steps  

    param.link_init_detect = '3'; 
    param.link_study_area =  '1';  
    param.NO_CAR_DIST = 50;     % No car distance [m]

    % camera tracking
    param.track_enable  = true;   % on/off
    param.track_every   = 1;      
    param.track_halfbox = 60;     % [m] half box size (60 => 120m x 120m)
    param.track_pause   = 0.01;   


    %% 1. MPC Settings 
    param.N_pred = 50;          % Prediction Horizon [Step]
                                % Time Step [s]
    param.tf = 2000;            % Simout time
    
    %% 2-1. Vehicle Specs 
    param.mass_base = 1685;     % Empty Vehicle Mass [kg]
    param.mass_pass = 65;       % Passenger/Cargo Mass [kg]
    param.mass = param.mass_base + param.mass_pass; % Total Mass) [kg]
    
    param.R_tire = 0.33415;          % Tire Radius [m]
    param.gear_ratio = 7.981;   % Gear Ratio
    
    % Mass Factor
    param.mass_factor = 1 + 0.04 * param.gear_ratio + 0.0025 * param.gear_ratio^2;
    
    param.RRC = (0.0066+0.0077)/2; % Rolling Resistance Coefficient
    
    %% 2-2. Environment Specs 
    param.gravity = 9.81;       % Gravity [m/s^2]
    param.rho = 1.2;            % Air Density [kg/m^3]
    param.Cd = 0.288;           % Drag Coefficient
    param.Af = 2.45;            % Frontal Area [m^2]
    param.wind_sped = 2;        % Headwind Speed [m/s] 

    % Aerodynamic Drag Factor
    param.k_air = 0.5 * param.rho * param.Cd * param.Af;

    %% 3-1. Battery & Thermal Specs 
    % Battery hierarchy:
    %   Cell   -> Module(16s2p) -> Pack(6 modules in series) = 96s2p
    param.Np      = 2;         % Parallel cells per module
    param.Ns_mod  = 16;         % Series cells per module
    param.N_mod_s = 6;          % Series modules per pack

    param.N_series_total = param.Ns_mod * param.N_mod_s;
    param.V_nom = 360;          % Battery Nominal Voltage [V]

    % Module-level internal resistance fit:
    %   R_mod(T[degC], Ah_use[Ah]) = (a*exp(-b*T)+c)*Ah_use^d
    % The fit is for one module (16s24p), so string/pack resistance is
    % module resistance multiplied by the number of series modules.
    loaded_params = load('AgingModelSet.mat');
    param.Fit_IR = loaded_params.Fit_IR;
    param.ir_ah_floor = 0.05;

    T_init_C = 25;
    Ah_use0 = param.ir_ah_floor;
    param.R_mod0 = feval(param.Fit_IR, T_init_C, Ah_use0);
    param.R_pack0 = param.R_mod0 * param.N_mod_s;

    % Backward-compatible alias mapped from the fitted module/string model
    param.R_cell0 = param.R_pack0 * param.Np / param.N_series_total;
    %% 3-2. Thermal Management Specs
    % BTMS: single-reference heating/cooling control
    param.k_i = 1.0;
    param.P_0 = 700;            % [W] minimum electrical cooling power once active
    param.Kp_T = 450;           % [W/K] cooling gain
    param.Pheat_0 = 400;        % [W] minimum electrical heating power once active
    param.Kp_heat = 350;        % [W/K] heating gain
    param.T_deadband = 0.1;     % [K]
    param.COP = 6.5;            % legacy alias
    param.Q_bat_nom = 120;      % Pack nominal capacity [Ah]
    param.Q_bat0    = param.Q_bat_nom * 0.8; % SOC 80%

    param.R_gas    = 8.314;         %[J/mol/k]
    param.theta = 298;

    param.T_amb   = 25 + 273.15;   % [K]
    param.T_init  = T_init_C + 273.15;   % [K]
    param.T_ref   = 30 + 273.15;   % [K] single BTMS reference temperature
    param.T_batt_max = 45 + 273.15; % [K]
    param.T_batt_min = 15 + 273.15; % [K]
    param.hA      = 10;             % [W/K] disable passive heat rejection for sensitivity check
    param.eta_cool = 1.5;          % cooling COP [Wth/We]
    param.COP_heat = 2.8;          % heating COP [Wth/We]
    param.Pcool_max = 5000;        % [W]
    param.Pheat_max = 5000;        % [W]
    param.Ptms_max  = max(param.Pcool_max, param.Pheat_max);

    param.batmass = 300;            % kg
    param.cp = 900;                 % [J/kg/K]
    param.Cth  = param.batmass * param.cp;  %[J/K]

    % Backward-compatible aliases used by other scripts
    param.Q_max_Ah = param.Q_bat_nom;
    param.Q_init0  = param.Q_bat0;
    param.R_str0   = param.R_pack0;
    param.R_int0   = param.R_cell0;  % backward-compatible alias
    param.Npar     = param.Np;
    param.Nser     = param.N_mod_s;


    %% 4. Constraints
    param.P_max = 120E3;       % Max Power [W] (+)
    param.P_min = -120E3;      % Max Regen Power [W] (-)

    param.Q_max = 5000;         % [W]                                                    
    
    % Distance Tolerance
    param.eps_dist = 5; 
    
    param.dist_tol = 1;
    param.acc_tol = 3.65; 

    %% 5. Aging Objective Settings
    % Onori-style battery life optimization:
    % Qloss%% -> gamma -> sigma -> sigma * |I|
    param.age_soc_ref = 0.5;                  % nominal SOC for aging normalization
    param.age_crate_ref = 1.0;                % nominal cell C-rate for aging normalization
    param.age_temp_ref = 25 + 273.15;         % nominal cell temperature [K]
    param.age_qref_pct = 20;                  % end-of-life capacity loss [%]
    param.age_exp = 0.57;                     % Ah-throughput exponent
    param.obj_w_trac = 0.7;                   % traction share in energy term
    param.obj_w_tms = 0.3;                    % BTMS share in energy term
    param.obj_w_cool = param.obj_w_tms;       % backward-compatible alias

    %% oad Power Model (BP_system)
    loaded_power = load('BP_system.mat');
    loaded_current = load('BC_sys.mat');
    loaded_ocv_mu  = load('OCV_mu.mat');
    loaded_ocv_sys = load('OCV_sys.mat');

    param.BPsys = loaded_power.BP_sys;  
    param.BCsys = loaded_current.BC_sys;
    param.OCV_mu = loaded_ocv_mu.mu;  
    param.OCV_sys = loaded_ocv_sys.p;

end
