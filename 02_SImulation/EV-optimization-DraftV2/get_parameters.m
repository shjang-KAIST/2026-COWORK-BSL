function param = get_parameters()
    % GET_CONFIG: 
    % Vissim setup parameters
    param.total_flow = 10;    % Total Flow [veh/hr]
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
    param.Np = 50;              % Prediction Horizon [Step]
                                % Time Step [s]
    param.tf = 2000;            % Simout time
    
    %% 2-1. Vehicle Specs 
    param.mass_base = 1685;     % Empty Vehicle Mass [kg]
    param.mass_pass = 65;       % Passenger/Cargo Mass [kg]
    param.mass = param.mass_base + param.mass_pass; % Total Mass) [kg]
    
    param.R = 0.33415;          % Tire Radius [m]
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
    param.V_nom = 360;          % Battery Nominal Voltage [V] 
    param.R_str0 = 0.0162;        % Battery Ohmic Resistance [Ohm] 
    param.Npar = 2;
    param.Nser = 6;
    %% 3-2. Thermal Management Specs
    % P_cool = P_base + k * Heat_Loss
    param.k_i = 1.0;    
    param.P_0 = 200;            % [W]
    % COP: 6.5
    param.COP = 6.5;    % [W/W]
    param.Q_init0  = 120;
    param.Q_max_Ah   = 120;

    param.R    = 8.314;
    param.theta = 298;

    %% 4. Constraints
    param.P_max = 120E3;       % Max Power [W] (+)
    param.P_min = -120E3;      % Max Regen Power [W] (-)

    param.Q_max = 5000;         % [W]                                                    
    
    % Distance Tolerance
    param.eps_dist = 5; 
    
    param.dist_tol = 3;
    param.acc_tol = 3.65; 


    %% 4. Load Power Model (BP_system)
        loaded_power = load('BP_system.mat');
        loaded_current = load('BC_sys.mat');
        loaded_ocv_mu  = load('OCV_mu.mat');
        loaded_ocv_sys = load('OCV_sys.mat');

        param.BPsys = loaded_power.BP_sys;  
        param.BCsys = loaded_current.BC_sys;
        param.OCV_mu = loaded_ocv_mu.mu;  
        param.OCV_sys = loaded_ocv_sys.p;

end
