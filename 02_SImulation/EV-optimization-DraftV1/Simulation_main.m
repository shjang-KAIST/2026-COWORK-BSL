clear; clc; close all;

%% 1. Config & Init params
param = get_parameters();

dt = param.dt;
Np = param.Np;
tf = param.tf;                  % total control duration (sec) in your design
tf_step = round(tf / dt);       % total steps to run after Fixed_steps

%% 2. VISSIM Connection
feature('COM_SafeArraySingleDim', 1);
try
    Vissim = actxGetRunningServer('Vissim.Vissim');
    disp('>> Vissim에 연결되었습니다.');
catch
    Vissim = actxserver('Vissim.Vissim');
    disp('>>  Vissim을 실행했습니다.');
end

Path_network = cd;
Filename = fullfile(Path_network, 'Matlab_simul.inpx');

if Vissim.Simulation.AttValue('IsRunning')
    Vissim.Simulation.Stop;
end
Vissim.LoadNet(Filename, false);

% (Optional) Speed up graphics
try
    win = Vissim.Graphics.CurrentNetworkWindow;
    set(win,'AttValue','QuickMode', 0);
catch
end

%% 3. VISSIM Scenario Setup
Link_number = 1;
set(Vissim.Net.Links.ItemByKey(Link_number), 'AttValue', 'Name', 'Study Area');
set(Vissim.Net.Links.ItemByKey(Link_number), 'AttValue','BehaviorType',1);

Rel_Flows = Vissim.Net.VehicleCompositions.ItemByKey(1).VehCompRelFlows.GetAll;
set(Rel_Flows{1}, 'AttValue', 'DesSpeedDistr', 120);
set(Rel_Flows{2}, 'AttValue', 'DesSpeedDistr', 100);

RSA = Vissim.Net.ReducedSpeedAreas.GetAll;
set(RSA{1}, 'AttValue','DesSpeedDistr(10)',30);
set(RSA{2}, 'AttValue','DesSpeedDistr(10)',30);
set(RSA{1}, 'AttValue','DesSpeedDistr(20)',25);
set(RSA{2}, 'AttValue','DesSpeedDistr(20)',25);
set(RSA{1}, 'AttValue','DesSpeedDistr(30)',20);
set(RSA{2}, 'AttValue','DesSpeedDistr(30)',20);

set(RSA{3}, 'AttValue','DesSpeedDistr(10)',30);
set(RSA{4}, 'AttValue','DesSpeedDistr(10)',30);
set(RSA{3}, 'AttValue','DesSpeedDistr(20)',25);
set(RSA{4}, 'AttValue','DesSpeedDistr(20)',25);
set(RSA{3}, 'AttValue','DesSpeedDistr(30)',20);
set(RSA{4}, 'AttValue','DesSpeedDistr(30)',20);

Total_flow = 7200;
set(Vissim.Net.VehicleInputs.ItemByKey(1), 'AttValue', 'Volume(1)', Total_flow*0.7);
set(Vissim.Net.VehicleInputs.ItemByKey(2), 'AttValue', 'Volume(1)', Total_flow*0.06);
set(Vissim.Net.VehicleInputs.ItemByKey(3), 'AttValue', 'Volume(1)', Total_flow*0.03);
set(Vissim.Net.VehicleInputs.ItemByKey(4), 'AttValue', 'Volume(1)', Total_flow*0.21);

%% 4. Data Buffers & Map
map_table = readtable('StudyArea_SlopeMap.csv');
degmap = table2array(map_table);

history = struct('opt_P_total',[], 'pred_P',[], 'opt_P_batt',[], ...
                 'opt_v',[], 'pred_v',[], ...
                 'opt_a',[], 'pred_a',[], ...
                 'opt_dist',[], 'pred_dist',[], ...
                 'opt_I',[], 'pred_I',[], ...
                 'opt_Q', [], ...
                 'opt_Qgen',[], 'pred_Qgen',[], ...   
                 'bound_v_max',[], 'bound_v_min',[], ...
                 'bound_a_max',[], 'bound_a_min',[], ...
                 'pred_f_dist',[], 'pred_r_dist',[],...
                 'opt_f_dist',[], 'opt_r_dist',[],...
                 'applied_v',[],  'applied_lane',[] ...
                 );

%% 5. Simulation initialization
Random_Seed = 42;
set(Vissim.Simulation, 'AttValue', 'RandSeed', Random_Seed);
set(Vissim.Simulation, 'AttValue', 'SimRes', 1/dt);
set(Vissim.Simulation, 'AttValue', 'SimSpeed', 10);

Simul_steps = round(400/dt);
init = 1;

disp('>> Vissim Warm-up...');
while init < Simul_steps
    Vissim.Simulation.RunSingleStep;
    init = init + 1;
end

% ---------------- Target vehicle ----------------
disp('>> Detecting target vehicle on link 3...');
Target = -1;
while Target < 0
    All_Vehicles = Vissim.Net.Vehicles.GetAll;
    for v_idx = 1:length(All_Vehicles)
        veh_ego = All_Vehicles{v_idx};
        try
            lane_str = char(get(veh_ego, 'AttValue', 'Lane'));
        catch
            continue;
        end
        link_id  = extractBefore(lane_str, '-');
        if strcmp(link_id, '3')
            Target = get(veh_ego, 'AttValue', 'No');
            disp(['>> Target Vehicle ID: ' num2str(Target)]);
            break;
        end
    end
    if Target < 0
        Vissim.Simulation.RunSingleStep;
        init = init + 1;
    end
end

disp('>> Waiting for Study Area (link 1)...');
while true
    try
        veh_ego = Vissim.Net.Vehicles.ItemByKey(Target);
        lane_str = char(get(veh_ego, 'AttValue', 'Lane'));
        link_id  = extractBefore(lane_str, '-');
        if strcmp(link_id, '1')
            break;
        end
    catch
        disp('Target vehicle disappeared!'); return;
    end
    Vissim.Simulation.RunSingleStep;
    init = init + 1;
end

%% 6. Prediction loop 
disp('>> Rollout Started...');
Fixed_steps = init;                 % Warm-up 후 초기 Fixed_steps
Pred_speds  = zeros(Np+1, 9);
Simout      = Simul_steps + tf_step - 1;
SIM_STOP_FLAG = false;

NO_CAR_DIST = 50;

while ~SIM_STOP_FLAG

    % =========================================================
    % (0) 현재까지 applied 길이 (rewind에서 재현할 구간 길이)
    % =========================================================
    nApplied = 0;
    if isfield(history,'applied_v')
        nApplied = numel(history.applied_v);    % = (Fixed_steps - Simul_steps) 누적치가 정상
    end

    % =========================================================
    % (A) Prediction rollout (forward Np+1 steps)
    % =========================================================
    disp(['[PRED] Start | Fixed_steps = ' num2str(Fixed_steps)]);
    Pred_speds(:,:) = 0;   % 매 loop 초기화(오염 방지)

    for cnt = 1:Np+1

        % -------- target 확보 --------
        try
            veh_ego = Vissim.Net.Vehicles.ItemByKey(Target);
        catch
            disp('>> Target vehicle disappeared during prediction!');
            SIM_STOP_FLAG = true;
            break;
        end

        % -------- ego states --------
        acc_val = get(veh_ego, 'AttValue', 'Acceleration');
        pos_val = get(veh_ego, 'AttValue', 'Pos');
        spd_val = get(veh_ego, 'AttValue', 'Speed') / 3.6;  % m/s

        % -------- slope --------
        slope = get_slope(pos_val, degmap, 0);

        % (optional debug)
        if cnt == 1
            idx = find(degmap(:,1) >= floor(pos_val), 1);
            if isempty(idx), slope_degmap = NaN; else, slope_degmap = degmap(idx,2); end
            slope_interp = get_slope(pos_val, degmap, 0);
            fprintf('[SlopeChk] pos=%.2f | degmap=%.5f | interp=%.5f\n', pos_val, slope_degmap, slope_interp);
        end

        % =========================================================
        % Front/Rear vehicle (SAME LANE only)
        % =========================================================
        f_dist = inf; f_v = 0;
        r_dist = inf; r_v = 0;

        ego_lane = char(get(veh_ego,'AttValue','Lane'));
        ego_link = extractBefore(ego_lane,'-');

        All_Vehicles = Vissim.Net.Vehicles.GetAll;
        for i = 1:length(All_Vehicles)
            v = All_Vehicles{i};
            v_no = get(v,'AttValue','No');
            if v_no == Target, continue; end

            try
                v_lane = char(get(v,'AttValue','Lane'));
                if isempty(v_lane), continue; end
                if ~strcmp(v_lane, ego_lane), continue; end

                v_link = extractBefore(v_lane,'-');
                if ~strcmp(v_link, ego_link), continue; end

                v_pos = get(v,'AttValue','Pos');
                v_spd = get(v,'AttValue','Speed') / 3.6;
            catch
                continue;
            end

            if v_pos > pos_val
                d = v_pos - pos_val;
                if d < f_dist, f_dist = d; f_v = v_spd; end
            elseif v_pos < pos_val
                d = pos_val - v_pos;
                if d < r_dist, r_dist = d; r_v = v_spd; end
            end
        end

        if ~isfinite(f_dist), f_dist = NO_CAR_DIST; f_v = 0; end
        if ~isfinite(r_dist), r_dist = NO_CAR_DIST; r_v = 0; end

        Pred_speds(cnt,:) = [acc_val, slope, spd_val^2, pos_val, dt, f_dist, f_v, r_dist, r_v];

        % -------- rollout --------
        Vissim.Simulation.RunSingleStep;

        % -------- termination --------
        if Fixed_steps + cnt >= Simout
            SIM_STOP_FLAG = true;
            disp('>> Simulation end reached during prediction!');
            break;
        end
    end

    if SIM_STOP_FLAG
        disp('>> Exiting main loop after prediction.');
        break;
    end

    % =========================================================
    % (B) Stop & rewind back to Fixed_steps
    %     - 1~Simul_steps: baseline
    %     - Simul_steps+1~Fixed_steps: 과거 replay에서 주입한 applied_v/lane 재현
    % =========================================================
    Vissim.Simulation.Stop;
    disp(['[Rewind] Rewinding to Fixed_steps = ' num2str(Fixed_steps)]);
    
    for init_rewind = 1:Fixed_steps
    
        if init_rewind <= Simul_steps
            Vissim.Simulation.RunSingleStep;
    
        else
            idx = init_rewind - Simul_steps;   % 1,2,3,...
            veh_ego = [];

            try
                veh_ego = Vissim.Net.Vehicles.ItemByKey(Target);
            catch
                veh_ego = [];
            end
    
            if isempty(veh_ego)
                Vissim.Simulation.RunSingleStep;
                continue;
            end
    
            if idx <= nApplied
                v_cmd = max(history.applied_v(idx), 0);   % m/s
                set(veh_ego, 'AttValue', 'Speed', v_cmd*3.6);
    
                if isfield(history,'applied_lane') && numel(history.applied_lane) >= idx ...
                        && ~isnan(history.applied_lane(idx))
                    set(veh_ego, 'AttValue', 'DesLane', int32(history.applied_lane(idx)));
                end
            end
    
            Vissim.Simulation.RunSingleStep;
        end
    end

    if SIM_STOP_FLAG, break; end

    % =========================================================
    % (C) MPC Optimization
    % =========================================================
    disp('[MPC] Computing optimal speeds profile ...');
    OPTout = get_Optspeeds(Pred_speds, Np, degmap);

    % =========================================================
    % (D) Data Logging 
    % =========================================================
    range = 1:Np;
    history.opt_P_total  = [history.opt_P_total;     OPTout.opt.P_total(range)];
    history.opt_P_batt   = [history.opt_P_batt;      OPTout.opt.P_batt(range)];
    history.pred_P       = [history.pred_P;          OPTout.pred.P(range)];
    history.opt_v        = [history.opt_v;           OPTout.opt.v(range)];
    history.pred_v       = [history.pred_v;          OPTout.pred.v(range)];
    history.opt_a        = [history.opt_a;           OPTout.opt.acc(range)];
    history.pred_a       = [history.pred_a;          OPTout.pred.acc(range)];
    history.bound_a_max  = [history.bound_a_max;     OPTout.bound.a_max(range)];
    history.bound_a_min  = [history.bound_a_min;     OPTout.bound.a_min(range)];
    history.opt_dist     = [history.opt_dist;        OPTout.opt.pos(range)];
    history.pred_dist    = [history.pred_dist;       OPTout.pred.pos(range)];
    history.opt_Q        = [history.opt_Q;           OPTout.opt.Q(range)];
    history.opt_I        = [history.opt_I;           OPTout.opt.I(range)];
    history.pred_I       = [history.pred_I;          OPTout.pred.I_batt(range)];
    history.pred_f_dist  = [history.pred_f_dist;     Pred_speds(1:Np,6)];
    history.pred_r_dist  = [history.pred_r_dist;     Pred_speds(1:Np,8)];
    history.bound_v_max  = [history.bound_v_max;     OPTout.bound.v_max(range)];
    history.bound_v_min  = [history.bound_v_min;     OPTout.bound.v_min(range)];
    fprintf('[Logging] Completed for Fixed_steps = %d\n', Fixed_steps);

    % =========================================================
    % (E) Replay control: run Np steps and set speed each step
    % =========================================================
    disp('[Replay] Applying optimal speeds to target vehicle...');
    set(Vissim.Simulation, 'AttValue', 'SimSpeed', 1);

    for k = 1:Np

        try
            veh_ego = Vissim.Net.Vehicles.ItemByKey(Target);
        catch
            disp('>> Target vehicle disappeared during replay!');
            SIM_STOP_FLAG = true;
            break;
        end

        track_target(Vissim, Target, 50);
        pause(0.01);

        % -------- lane change candidate --------
        ego_lane_str = char(get(veh_ego,'AttValue','Lane'));
        ego_link_str = extractBefore(ego_lane_str,'-');
        ego_lane_idx = str2double(extractAfter(ego_lane_str,'-'));

        try
            linkObj = Vissim.Net.Links.ItemByKey(str2double(ego_link_str));
            nLanes  = double(linkObj.AttValue('NumLanes'));
            if ~isfinite(nLanes) || nLanes < 1
                nLanes = max(ego_lane_idx,2);
            end
        catch
            nLanes = max(ego_lane_idx,2);
        end

        laneCandidates = unique([ego_lane_idx-1, ego_lane_idx, ego_lane_idx+1]);
        laneCandidates = laneCandidates(laneCandidates >= 1 & laneCandidates <= nLanes);

        infoLane = lane_gaps(Vissim, veh_ego, Target, ego_link_str, laneCandidates, NO_CAR_DIST);
        bestLane = choose_best_lane(infoLane, param.dist_tol, 1);

        % lane apply
        set(veh_ego,'AttValue','DesLane', int32(bestLane));

        % speed apply 
        v_cmd = max(OPTout.opt.v(k), 0);
        set(veh_ego, 'AttValue', 'Speed', v_cmd*3.6);

        % =========================================================
        % (A) f_dist / r_dist 
        % =========================================================
        f_dist = inf; r_dist = inf;
        
        ego_lane_now = char(get(veh_ego,'AttValue','Lane'));
        ego_link_now = extractBefore(ego_lane_now,'-');
        ego_pos_now  = double(get(veh_ego,'AttValue','Pos'));
        
        All_Vehicles = Vissim.Net.Vehicles.GetAll;
        for ii = 1:length(All_Vehicles)
            v = All_Vehicles{ii};
            v_no = get(v,'AttValue','No');
            if v_no == Target, continue; end
        
            try
                v_lane = char(get(v,'AttValue','Lane'));
                if isempty(v_lane), continue; end
        
                % SAME LANE + SAME LINK
                if ~strcmp(v_lane, ego_lane_now), continue; end
                v_link = extractBefore(v_lane,'-');
                if ~strcmp(v_link, ego_link_now), continue; end
        
                v_pos = double(get(v,'AttValue','Pos'));
            catch
                continue;
            end
        
            if v_pos > ego_pos_now
                d = v_pos - ego_pos_now;
                if d < f_dist, f_dist = d; end
            elseif v_pos < ego_pos_now
                d = ego_pos_now - v_pos;
                if d < r_dist, r_dist = d; end
            end
        end
        
        if ~isfinite(f_dist), f_dist = NO_CAR_DIST; end
        if ~isfinite(r_dist), r_dist = NO_CAR_DIST; end

        % ===== Data Logging =====
        history.applied_v    = [history.applied_v; v_cmd];       % m/s
        history.applied_lane = [history.applied_lane; bestLane]; % optional
        history.opt_f_dist   = [history.opt_f_dist; f_dist];
        history.opt_r_dist   = [history.opt_r_dist; r_dist];
       
        % ======================================

        fprintf('[REPLAY] k=%d | pos=%.1f | LaneNow=%s | DesLane=%d | v_cmd=%.2f m/s\n', ...
            k, double(get(veh_ego,'AttValue','Pos')), ego_lane_str, int32(get(veh_ego,'AttValue','DesLane')), v_cmd);

        % rollout
        Vissim.Simulation.RunSingleStep;

        Fixed_steps = Fixed_steps + 1;

        if Fixed_steps >= Simout
            SIM_STOP_FLAG = true;
            disp('>> Simulation end reached during replay!');
            break;
        end
    end

    set(Vissim.Simulation, 'AttValue', 'SimSpeed', 10);
    fprintf('[Loop End] Fixed_steps=%d / Simout=%d\n', Fixed_steps, Simout);

end

Vissim.Simulation.Stop;
disp('>> Simulation finished.');


%% 7. Visualization
plot_results(history,dt);

function plot_results(h, dt)

    % =========================================================
    % Plot settings
    % =========================================================
    lw = 1.5; 
    fs = 14; 
    fn = 'Times New Roman';
    N = length(h.opt_v);
    t = (0:N-1)' * dt;   % [s]

    % =========================================================
    % Power
    % =========================================================
    figure('Name','Power Profile','Color','w');
    plot(t, h.opt_P_total, 'LineWidth', lw); hold on;
    plot(t, h.pred_P, '--', 'LineWidth', lw);
    plot(t, h.opt_P_batt, ':', 'LineWidth', lw);
    legend({'Opt P_{total}','Baseline P','Opt P_{batt}'}, 'Location','northeast');
    xlabel('Time [s]'); ylabel('Power [W]');
    title('Power Profile');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Speed (Opt vs Pred)
    % =========================================================
    figure('Name','Speed Profile','Color','w');
    plot(t, h.opt_v * 3.6, 'LineWidth', lw); hold on;
    plot(t, h.pred_v * 3.6, '--', 'LineWidth', lw);
    legend({'Optimal','Baseline'}, 'Location', 'southeast');
    xlabel('Time [s]'); ylabel('Speed [km/h]');
    title('Vehicle Speed Profile');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Speed (Opt + Bounds)
    % =========================================================
    figure('Name','Speed Profile with Bounds','Color','w');
    plot(t, h.opt_v * 3.6, 'LineWidth', lw); hold on;
    plot(t, h.bound_v_max * 3.6, ':', 'LineWidth', 1.2);
    plot(t, h.bound_v_min * 3.6, ':', 'LineWidth', 1.2);
    legend({'Optimal','v_{max} bound','v_{min} bound'}, 'Location','southeast');
    xlabel('Time [s]'); ylabel('Speed [km/h]');
    title('Vehicle Speed Profile (with bounds)');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Distance
    % =========================================================
    figure('Name','Distance Traveled','Color','w');
    plot(t, h.opt_dist/1000, 'LineWidth', lw); hold on;
    plot(t, h.pred_dist/1000, '--', 'LineWidth', lw);
    legend({'Optimal','Baseline'}, 'Location', 'southeast');
    xlabel('Time [s]'); ylabel('Distance [km]');
    title('Cumulative Distance');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Heat (Loss surrogate)
    % =========================================================
    figure('Name','Heat Generation','Color','w');
    plot(t, h.opt_Q, 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('Heat surrogate [W]');
    title('Thermal Generation (Loss / Surrogate)');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % ---- Cumulative heat (Wh-eq) ----
    figure('Name','Cumulative Heat','Color','w');
    plot(t, cumsum(h.opt_Q(:))*dt/3600, 'LineWidth', lw);
    xlabel('Time [s]'); ylabel('Heat [Wh-eq]');
    title('Cumulative Heat Surrogate');
    grid on; set(gca,'FontSize',fs,'FontName',fn);

    % =========================================================
    % Acceleration + Bounds
    % =========================================================
    figure('Name','Acceleration Constraints','Color','w');
    plot(t, h.opt_a, 'LineWidth', lw); hold on;
    plot(t, h.bound_a_max, '--', 'LineWidth', 1.2);
    plot(t, h.bound_a_min, '--', 'LineWidth', 1.2);
    legend({'Optimal Acc','Upper Bound','Lower Bound'}, 'Location', 'northeast');
    xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
    title('Acceleration vs. Safety Bounds');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Battery Current
    % =========================================================
    figure('Name','Battery Current','Color','w');
    plot(t, h.opt_I, 'LineWidth', lw); hold on;
    plot(t, h.pred_I, '--', 'LineWidth', lw);
    legend({'Optimal','Baseline'}, 'Location', 'northeast');
    xlabel('Time [s]'); ylabel('Current [A]');
    title('Battery Current');
    grid on; set(gca,'FontSize',fs,'FontName',fn);

    % ---- Cumulative Ah ----
    figure('Name','Cumulative Ah','Color','w');
    plot(t, cumsum(h.opt_I(:))*dt/3600, 'LineWidth', lw); hold on;
    plot(t, cumsum(h.pred_I(:))*dt/3600, '--', 'LineWidth', lw);
    legend({'Optimal','Baseline'}, 'Location', 'southeast');
    xlabel('Time [s]'); ylabel('Capacity [Ah]');
    title('Cumulative Current (Ah)');
    grid on; set(gca,'FontSize',fs,'FontName',fn);

    % =========================================================
    % Front Vehicle Distance (Opt vs Pred)
    % =========================================================
    Lf = min([length(t), length(h.opt_f_dist), length(h.pred_f_dist)]);
    figure('Name','Front Vehicle Distance','Color','w');
    plot(t(1:Lf), h.opt_f_dist(1:Lf), 'LineWidth', lw); hold on;
    plot(t(1:Lf), h.pred_f_dist(1:Lf), '--', 'LineWidth', lw);
    legend({'Opt Front','Pred Front'}, 'Location','northeast');
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Front Vehicle Distance');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Rear Vehicle Distance (Opt vs Pred)
    % =========================================================
    Lr = min([length(t), length(h.opt_r_dist), length(h.pred_r_dist)]);
    figure('Name','Rear Vehicle Distance','Color','w');
    plot(t(1:Lr), h.opt_r_dist(1:Lr), 'LineWidth', lw); hold on;
    plot(t(1:Lr), h.pred_r_dist(1:Lr), '--', 'LineWidth', lw);
    legend({'Opt Rear','Pred Rear'}, 'Location','northeast');
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Rear Vehicle Distance');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Energy (Wh)
    % =========================================================
    figure('Name','Energy Consumption','Color','w');
    plot(t, cumsum(h.opt_P_total(:))*dt/3600, 'LineWidth', lw); hold on;
    plot(t, cumsum(h.pred_P(:))*dt/3600, '--', 'LineWidth', lw);
    legend({'Optimal','Baseline'}, 'Location', 'southeast');
    xlabel('Time [s]'); ylabel('Energy [Wh]');
    title('Total Energy Consumption');
    grid on; set(gca, 'FontSize', fs, 'FontName', fn);

    % =========================================================
    % Summary
    % =========================================================
    opt_energy  = sum(h.opt_P_total(:))*dt/3600;
    base_energy = sum(h.pred_P(:))*dt/3600;
    save_ratio  = (base_energy - opt_energy) / max(base_energy,1e-9) * 100;

    fprintf('\n================ RESULT ================\n');
    fprintf('Baseline Energy: %.2f Wh\n', base_energy);
    fprintf('Optimal Energy : %.2f Wh\n', opt_energy);
    fprintf('Energy Saving  : %.2f %%\n', save_ratio);
    fprintf('========================================\n');

end

function track_target(Vissim, Target, halfBox_m)
% track_target_zoomto
% - Follow target vehicle in 2D by calling NetworkWindow.ZoomTo()
% - halfBox_m: half size of view box [m]. Smaller => more zoom-in.

if nargin < 3, halfBox_m = 50; end   % 50m half-box = 100m x 100m view

% 1) target coord from CoordFront: 'X Y Z'
    try
        veh = Vissim.Net.Vehicles.ItemByKey(Target);
        s   = get(veh,'AttValue','CoordFront');     % ex) '706.720 785.471 0.000'
    catch
        return;
    end

    xyz = sscanf(char(s),'%f');
    if numel(xyz) < 2, return; end
    x = xyz(1);  y = xyz(2);

% 2) network window
    try
        win = Vissim.Graphics.CurrentNetworkWindow;
    catch
        return;
    end

% 3) move/zoom view to a bounding box around (x,y)
    xmin = x - halfBox_m;
    xmax = x + halfBox_m;
    ymin = y - halfBox_m;
    ymax = y + halfBox_m;

    try
        invoke(win, 'ZoomTo', xmin, ymin, xmax, ymax);
    catch
        try
            win.ZoomTo(xmin, ymin, xmax, ymax);
        catch
        end
    end
end

function info = lane_gaps(Vissim, egoVeh, Target, linkId, laneIdxList, NO_CAR_DIST)
% info(l).lane, front_gap, front_v, rear_gap, rear_v

if nargin < 6, NO_CAR_DIST = 50; end

egoPos = get(egoVeh,'AttValue','Pos');
allVeh = Vissim.Net.Vehicles.GetAll;

info = struct('lane',{},'front_gap',{},'front_v',{},'rear_gap',{},'rear_v',{});

    for ii = 1:numel(laneIdxList)
        ln = laneIdxList(ii);
        f_dist = inf; f_v = 0;
        r_dist = inf; r_v = 0;
    
        for i = 1:length(allVeh)
            v = allVeh{i};
            v_no = get(v,'AttValue','No');
            if v_no == Target, continue; end
    
            try
                v_lane_str = char(get(v,'AttValue','Lane'));  % e.g. "1-1"
                if isempty(v_lane_str), continue; end
                v_link = extractBefore(v_lane_str,'-');
                v_lane = str2double(extractAfter(v_lane_str,'-'));  % lane index
                if ~strcmp(v_link, linkId), continue; end
                if v_lane ~= ln, continue; end
    
                v_pos = get(v,'AttValue','Pos');
                v_spd = get(v,'AttValue','Speed')/3.6;
            catch
                continue;
            end
    
            if v_pos > egoPos
                d = v_pos - egoPos;
                if d < f_dist
                    f_dist = d;
                    f_v = v_spd;
                end
            elseif v_pos < egoPos
                d = egoPos - v_pos;
                if d < r_dist
                    r_dist = d;
                    r_v = v_spd;
                end
            end
        end
    
        if ~isfinite(f_dist), f_dist = NO_CAR_DIST; f_v = 0; end
        if ~isfinite(r_dist), r_dist = NO_CAR_DIST; r_v = 0; end
    
        info(ii).lane = ln;
        info(ii).front_gap = f_dist;
        info(ii).front_v   = f_v;
        info(ii).rear_gap  = r_dist;
        info(ii).rear_v    = r_v;
    end
end

function bestLane = choose_best_lane(info, dist_tol, rear_min)
% info: struct array from lane_gaps()
%   info(i).lane
%   info(i).front_gap
%   info(i).rear_gap
%
% dist_tol: minimum desired front distance
% rear_min: minimum safe rear distance (e.g., 10~15m)

if nargin < 3
    rear_min = 12;   % default
end

bestLane  = info(1).lane;
bestScore = -inf;

    for i = 1:numel(info)
        fg = info(i).front_gap;
        rg = info(i).rear_gap;
    
        % ---------------------------
        % Hard safety exclusion
        % ---------------------------
        if rg < rear_min
            continue;   
        end
    
        % ---------------------------
        % Scoring
        % ---------------------------
        score = fg + 0.2 * rg;   
    
        % front gap 
        if fg < dist_tol
            score = score - 1000;
        end
    
        % ---------------------------
        % Best update
        % ---------------------------
        if score > bestScore
            bestScore = score;
            bestLane  = info(i).lane;
        end
    end

end
