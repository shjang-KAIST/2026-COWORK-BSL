function SIM_STOP_FLAG = VissimRewind(Vissim, Target, Fixed_steps, Simul_steps, history)
%VISSIMREWIND Rebuild the simulation up to Fixed_steps using app commands.

SIM_STOP_FLAG = false;

try
    Vissim.Simulation.Stop;
catch
end

fprintf('[VissimRewind] >> Rewinding to Fixed_steps = %d\n', Fixed_steps);

app_v = [];
if nargin >= 5 && isstruct(history) && isfield(history,'app') && isstruct(history.app)
    if isfield(history.app,'v') && ~isempty(history.app.v)
        app_v = history.app.v(:);
    end
end

for init_rewind = 1:Fixed_steps

    if init_rewind <= Simul_steps
        try
            Vissim.Simulation.RunSingleStep;
        catch
            SIM_STOP_FLAG = true;
            return;
        end
        continue;
    end

    idx = init_rewind - Simul_steps;

    try
        veh_ego = Vissim.Net.Vehicles.ItemByKey(int32(Target));
    catch
        veh_ego = [];
    end

    if ~isempty(veh_ego) && idx >= 1 && idx <= numel(app_v)
        v_cmd = app_v(idx);
        if isfinite(v_cmd)
            v_cmd = max(v_cmd, 0);
            try
                set(veh_ego, 'AttValue', 'Speed', v_cmd*3.6);
            catch
            end
        end
    end

    try
        Vissim.Simulation.RunSingleStep;
    catch
        SIM_STOP_FLAG = true;
        return;
    end
end

end
