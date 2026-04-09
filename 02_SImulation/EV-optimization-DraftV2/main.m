clear; clc; close all;
feature('COM_SafeArraySingleDim', 1);             % COM Interface

%% (1) Parameters
param = get_parameters();                         % Vissim Simulation & parameter load

%% (2) VISSIM init + scenario
Vissim = VissimInitconfig('Matlab_simul.inpx');   % VISSIM network load
VissimSetupScenario(Vissim, param);               % VISSIM Scenario configuration

%% (3) Slope map
table_map = readtable('StudyArea_SlopeMap.csv');  % Slope map load
degmap = table2array(table_map);

%% (4) Simulation Warm-up for the target vehicle detection
[TargetID, Simul_steps] = VissimRunInit(Vissim, param);  % Targe ID detection 
%% (5) MPC loop state
history       = get_HistoryInit();                           % Log struct
Fixed_steps   = Simul_steps;                                 % Current simulatio step
Simout        = Simul_steps + round(param.tf/param.dt) - 1;  % Simulation termination step
SIM_STOP_FLAG = false;
cnt = 0;

% ---- Init battery state ----
batt.Q_Ah   = param.Q_init0;           % Initial battey capacity
batt.I_prev = 0;
batt.Ah_th  = 0;
batt.R_str0 = param.R_str0;            % Initial battey resistance

while ~SIM_STOP_FLAG

    % Predict Np+1 window 
    [Pred_speds, SIM_STOP_FLAG] = VissimRollout(Vissim, TargetID, param, degmap);
    if SIM_STOP_FLAG, break; end

    % Rewind back to current step 
    SIM_STOP_FLAG = VissimRewind(Vissim, TargetID, Fixed_steps, Simul_steps);
    if SIM_STOP_FLAG, break; end

    % Optimize 
    OPTout = get_Optspeeds(Pred_speds, param, degmap, batt);

    % History log 
    history = get_HistoryLog(history, Pred_speds, [], OPTout, param, Fixed_steps, batt, []);

    % Apply Np-step opt & update opt and base battery state
    [history, Fixed_steps, SIM_STOP_FLAG, batt] = ...
        VissimReplayOpt(Vissim, TargetID, param, OPTout, Fixed_steps, Simout, history, degmap, batt);

    cnt = cnt + 1;

    if mod(cnt, 3) == 0
        get_plot(history, param);
        drawnow;
    end

    fprintf('[Window] >> Prediction window = %d\n', cnt);
end

close all;
Vissim.Simulation.Stop;  
get_plot(history, param);
disp('>> Simulation finished.\n');
