function [TargetID, Simul_steps] = VissimRunInit(Vissim, param)

    %% 1. Simulation Warm-up Phase


    Simul_steps = 0;
    Warmup_step = param.warmup_step;

    fprintf('\n');
    fprintf('======================================================\n');
    fprintf('      [VissimRunInit] Warm-up 진행 중... (%d steps) \n', Warmup_step);
    fprintf('======================================================\n');

    LINK_INIT_DETECT = param.link_init_detect; 
    LINK_STUDY_AREA  = param.link_study_area;  
    

    while Simul_steps <= Warmup_step
        Vissim.Simulation.RunSingleStep;
        Simul_steps = Simul_steps + 1;
    end 
    fprintf('>> Warm-up 완료. (Simul Step: %d)\n', Simul_steps);

    %% 2. Target Vehicle Dectection Phase in Link3
    fprintf('>> Link %s 에서 타겟 차량 탐색 시작.\n', LINK_INIT_DETECT);

    TargetID = -1;

    while TargetID < 0
        All_Vehicles = Vissim.Net.Vehicles.GetAll;

        for v_idx = 1:length(All_Vehicles)
            veh_ego = All_Vehicles{v_idx};

            try 
                lane_str = char(get(veh_ego, 'AttValue', 'Lane'));
                cur_link = extractBefore(lane_str, '-');

                if strcmp(cur_link, LINK_INIT_DETECT)           % String compare
                    TargetID = get(veh_ego, 'AttValue', 'No');
                    fprintf('>> Link %s 에서 타겟 차량 발견! (ID: %d)\n', LINK_INIT_DETECT, TargetID);
                    break;
                end
            catch
                continue;
            end
        end

        if TargetID < 0
            Vissim.Simulation.RunSingleStep;
            Simul_steps = Simul_steps + 1;
        end
    end
    %% 3. Target Vehicle Waiting Phase in Study Area Link 1
    fprintf('>> Link %s 에서 타겟 차량 탐색 시작.\n', LINK_STUDY_AREA);
    
    while true
        try
            veh_ego = Vissim.Net.Vehicles.ItemByKey(TargetID);
            lane_str = char(get(veh_ego, 'AttValue', 'Lane'));
            cur_link = extractBefore(lane_str, '-');

            if strcmp(cur_link, LINK_STUDY_AREA)           
                fprintf('>> Link %s 에서 타겟 차량 발견! (ID: %d)\n', LINK_STUDY_AREA, TargetID);
                break;
            end
        catch
            warning('타겟 차량을 찾을 수 없습니다.');
            TargetID = -1;
            return;
        end

        Vissim.Simulation.RunSingleStep;
        Simul_steps = Simul_steps + 1;
    end


    fprintf('>> [VissimRunInit] Warm-up 완료.\n');
    fprintf('======================================================\n');
end
     