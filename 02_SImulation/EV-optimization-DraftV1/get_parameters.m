function param = get_parameters()
    % GET_CONFIG: 
    
    %% 1. MPC Settings 
    param.Np = 20;              % Prediction Horizon [Step]
    param.dt = 0.5;               % Time Step [s]
    param.tf = 3000;
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
    param.R_batt = 0.05;        % Battery Ohmic Resistance [Ohm] 
    
    %% 3-2. Thermal Management Specs
    % P_cool = P_base + k * Heat_Loss
    param.k_i = 0.000875;    
    param.P_0 = 200;            % [W]
    % COP: 6.5
    param.cool_coeff = 0.15;    % [W/W]
    %% 4. Constraints
    param.P_max = 120E3;       % Max Power [W] (+)
    param.P_min = -120E3;      % Max Regen Power [W] (-)

    param.Q_max = 1500;         % [W]                                                    
    
    % Distance Tolerance
    param.eps_dist = 5; 
    
    param.dist_tol = 3;
    param.acc_tol = 3.65; 


    %% 4. Load Power Model (BP_system)
    try
        loaded = load('BP_system.mat'); 
        param.BPsys = loaded.BP_sys;  
        param.BPmodel = true;     
        disp('BP_system.mat 로드 완료: 전력 모델을 사용합니다.');
    catch
        warning('BP_system.mat 파일이 없습니다! 기본 물리 모델로 대체됩니다.');
        param.BPsys = [];
        param.BPmodel = false;
        
        param.Mot_par = [0.8671 -8.1; 0.7161 -4.245; 0.647 8.1805]; 
        param.regen_eff = 0.3; 
    end
end