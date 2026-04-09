function SIM_STOP_FLAG = VissimRewind(Vissim, Target, Fixed_steps, Simul_steps, history)
% VissimRewind

SIM_STOP_FLAG = false;

try
    Vissim.Simulation.Stop;
catch
end

fprintf('[VissimRewind] >> Rewinding to Fixed_steps = %d\n', Fixed_steps);

% ---- pull applied_v safely ----
applied_v = [];
if nargin >= 5 && isstruct(history) && isfield(history,'applied_v') && ~isempty(history.applied_v)
    applied_v = history.applied_v(:);
end

for init_rewind = 1:Fixed_steps

    % warmup section: no injection
    if init_rewind <= Simul_steps
        try
            Vissim.Simulation.RunSingleStep;
        catch
            SIM_STOP_FLAG = true;
            return;
        end
        continue;
    end

    idx = init_rewind - Simul_steps;   % 1,2,3,...

    try
        veh_ego = Vissim.Net.Vehicles.ItemByKey(int32(Target));
    catch
        veh_ego = [];
    end

    if ~isempty(veh_ego) && idx >= 1 && idx <= numel(applied_v)
        v_cmd = applied_v(idx);

        if isfinite(v_cmd)
            v_cmd = max(v_cmd, 0);   % m/s
            try
                set(veh_ego, 'AttValue', 'Speed', v_cmd*3.6);
            catch
            end
        end
    end

    % ---- simulation ----
    try
        Vissim.Simulation.RunSingleStep;
    catch
        SIM_STOP_FLAG = true;
        return;
    end
end

end
