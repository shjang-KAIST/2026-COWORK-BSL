clear; clc; close all;
feature('COM_SafeArraySingleDim', 1);             % COM Interface

%% -------------------- user option -------------------- %%
% FlagSweep = True;  % alpha, volume 전체 list 에 대한 for loop 동작
FlagSweep = false;   % alpha, volume list 1개에 대한 for loop 동작

% param.T_ref list 추가해서 for loop 동작시켜 clustering
% param.T_c 에서 초기 온도 조정

%%
% save path
save_path1 = 'C:\Users\user\Downloads\BSL_\1_연구\EREV에너지관리연구\Code\EV-optimization-DraftV2\Save';
save_path2 = 'C:\Users\Seunghun Jang\Desktop\Ongoing_win\2026-Cowork-Battery-thermal-energy\02_SImulation\EV-optimization-DraftV3\EV-optimization-DraftV2\fig_results';

% -------------------- VISSIM init --------------------
Vissim = VissimInitconfig('Matlab_simul.inpx');   % VISSIM network load

% -------------------- case lists --------------------
input_list = [1800; 3600; 5400; 6300];
alpha_list = 0.5;   %  [0.2; 0.5; 0.8]
%temp_list = [20; 30; 40];
% -------------------- sweep flag --------------------
if FlagSweep
    input_vec = input_list;
    alpha_vec = alpha_list;
else
    input_vec = input_list(1);
    alpha_vec = alpha_list(1);
end

% -------------------- save path select --------------------
if exist(save_path1, 'dir')
    save_path = save_path1;
else
    warning('Primary save path not found. Using fallback path.');
    save_path = save_path2;
end

if ~exist(save_path, 'dir')
    mkdir(save_path);
end

% -------------------- main loop --------------------
for i = 1:length(input_vec)
    input = input_vec(i);

    for j = 1:length(alpha_vec)
        alpha = alpha_vec(j);

        disp(['Start!, i: ' num2str(i) ', j: ' num2str(j)])

        %% (1) Parameters
        param = get_parameters(input);

        %% (2) VISSIM init + scenario
        VissimSetupScenario(Vissim, param);

        %% (3) Slope map
        table_map = readtable('StudyArea_SlopeMap.csv');
        degmap = table2array(table_map);

        %% (4) Simulation Warm-up for the target vehicle detection
        [TargetID, Simul_steps] = VissimRunInit(Vissim, param);

        %% (5) MPC loop state
        history       = get_HistoryInit();
        Fixed_steps   = Simul_steps;
        Simout        = Simul_steps + round(param.tf/param.dt) - 1;
        SIM_STOP_FLAG = false;
        cnt = 0;

        % ---- Init battery state ----
        batt.Q_bat  = param.Q_bat0;
        batt.Q_Ah   = param.Q_bat0;
        batt.I_prev = 0;
        batt.Ah_th  = 0;
        batt.Ah_eff = 0;
        batt.T      = param.T_init;
        batt.Rb     = param.R_pack0;
        batt.R_str0 = param.R_pack0;

        while ~SIM_STOP_FLAG

            % Predict Np+1 window
            [Pred_speds, SIM_STOP_FLAG] = VissimRollout(Vissim, TargetID, param, degmap);
            if SIM_STOP_FLAG, break; end

            % Rewind back to current step
            SIM_STOP_FLAG = VissimRewind(Vissim, TargetID, Fixed_steps, Simul_steps, history);
            
            if SIM_STOP_FLAG, break; end

            % Optimize
            OPTout = get_Optspeeds(Pred_speds, param, degmap, batt, alpha);

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
        disp('>> Simulation finished.');

        %% save
        filename = ['DATA_FLOW_' num2str(param.total_flow) ...
                    '_alpha' num2str(alpha*10) ...
                    '_inisoc0_8_without_resist_variation.mat'];

        save(fullfile(save_path, filename), 'history');
        fprintf('Saved: %s\n', fullfile(save_path, filename));
    end
end

%% example load
% history2 = load(fullfile(save_path, 'DATA_FLOW_7200.mat'));
