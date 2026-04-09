function VissimSetupScenario(Vissim, param)

    fprintf('\n'); % (
    fprintf('======================================================\n');
    fprintf('      [VissimSetup] 시나리오 파라미터 설정 중... \n');
    fprintf('======================================================\n');
    %% 1. Set Study Area of Link
    Link_num = 1;

    try
    LinkObj = Vissim.Net.Links.ItemByKey(Link_num);
    set(LinkObj, 'AttValue', 'Name', 'Study Area');
    set(LinkObj, 'AttValue', 'BehaviorType', 1);
    disp(['>> Link ', num2str(Link_num), ' 설정 완료.']);
    
    catch
        warning(['>> 오류: Link ', num2str(Link_num), '번을 찾을 수 없습니다. .inpx 파일을 확인하세요.']);
    end

    %% 2. Set Vehicle Input Flow
    % Composition(1) of Traffic Flow
    Rel_Flows = Vissim.Net.VehicleCompositions.ItemByKey(Link_num).VehCompRelFlows.GetAll;

    % Desired vehicle velocity distribution
    set(Rel_Flows{1}, 'AttValue', 'DesSpeedDistr', 120); % 120 [km/h]
    set(Rel_Flows{2}, 'AttValue', 'DesSpeedDistr', 100); % 100 [km/h]

    %% 3. Set Reduced Speed Area
    RSA = Vissim.Net.ReducedSpeedAreas.GetAll;

    for i = 1:4
        if i <= length(RSA)
            set(RSA{i}, 'AttValue', 'DesSpeedDistr(10)', 30); % 30 [km/h]
            set(RSA{i}, 'AttValue', 'DesSpeedDistr(20)', 25); % 25 [km/h]
            set(RSA{i}, 'AttValue', 'DesSpeedDistr(30)', 20); % 20 [km/h]
        else
            break;
        end
    end

    %% 4. Set Vehicle Input Volume
    Total_flow = param.total_flow; % Total Flow [veh/hr]
    flow_rates = [param.flow_rate1, param.flow_rate2, param.flow_rate3, param.flow_rate4];

    for i = 1:4
        VolObj = Total_flow * flow_rates(i);
        inputObj = Vissim.Net.VehicleInputs.ItemByKey(i);

        set(inputObj, 'AttValue', 'Volume(1)', VolObj); % Volume setting
    end

    %% 5. Set Simulation Parameters
    % 5.1 Simulation random seed
    Random_seed = param.random_seed;
    set(Vissim.Simulation, 'AttValue', 'RandSeed', Random_seed);

    % 5.2 Simulation time step
    dt = param.dt;               % Time Step [s]
    SimRes_val = round(1/dt);    % Simulation Resolution
    set(Vissim.Simulation, 'AttValue', 'SimRes', SimRes_val); 

    % 5.3 Simulation run time (default: 10)
    Sim_speed = param.sim_speed_10; % Simulation Speed
    set(Vissim.Simulation, 'AttValue', 'SimSpeed', Sim_speed);

    fprintf('>> Random Seed : %d\n', Random_seed);
    fprintf('>> Time Step   : %.1f [s] (Resolution: %d)\n', dt, SimRes_val);
    fprintf('>> Sim Speed   : %d\n', Sim_speed);
    fprintf('>> Flow1 vol   : %.1f [veh/hr]\n', flow_rates(1) * Total_flow);
    fprintf('>> Flow2 vol   : %.1f [veh/hr]\n', flow_rates(2) * Total_flow);
    fprintf('>> Flow3 vol   : %.1f [veh/hr]\n', flow_rates(3) * Total_flow);
    fprintf('>> Flow4 vol   : %.1f [veh/hr]\n', flow_rates(4) * Total_flow);
    fprintf('>> Total vol   : %.1f [veh/hr]\n',  Total_flow);
    fprintf('>> [VissimSetup] 시나리오 파라미터 설정 완료.\n');
    fprintf('======================================================\n');
end