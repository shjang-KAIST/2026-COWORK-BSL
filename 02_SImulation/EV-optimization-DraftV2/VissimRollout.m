function [Pred_speds, SIM_STOP_FLAG] = VissimRollout( ...
    Vissim, TargetID, param, degmap)

% Pred_speds (Nx x 10)
%   col1  acc        [m/s^2]
%   col2  slope      [-]
%   col3  spd^2      [(m/s)^2]
%   col4  pos        [m]
%   col5  dt         [s]
%   col6  f_dist     [m]
%   col7  r_dist     [m]
%   col8  f_v        [m/s]
%   col9  r_v        [m/s]
%   col10 ego_spd    [m/s]

Np = param.Np;
Nx = Np + 1;
dt = param.dt;
NO_CAR_DIST = param.NO_CAR_DIST;

Pred_speds = zeros(Nx,10);
SIM_STOP_FLAG = false;

% fast forward
try
    set(Vissim.Simulation, 'AttValue', 'SimSpeed', 10);
catch
end

Nvalid = 0;

for i = 1:Nx

    % ---- target ----
    try
        ego = Vissim.Net.Vehicles.ItemByKey(int32(TargetID));
    catch
        ego = [];
    end
    if isempty(ego)
        SIM_STOP_FLAG = true;
        break;
    end

    % ---- ego states ----
    ego_pos = double(get(ego,'AttValue','Pos'));                 % [m]
    ego_spd = double(get(ego,'AttValue','Speed')) / 3.6;        % [m/s]
    ego_acc = double(get(ego,'AttValue','Acceleration'));        % [m/s^2]

    % ---- slope ----
    slope = get_slope(ego_pos, degmap, 0);

    % ---- gaps (placeholder for now) ----
    f_dist = NO_CAR_DIST; f_v = 0;
    r_dist = NO_CAR_DIST; r_v = 0;

    % ---- pack ----
    Pred_speds(i,:) = [ ...
        ego_acc, ...
        slope, ...
        ego_spd^2, ...
        ego_pos, ...
        dt, ...
        f_dist, ...
        r_dist, ...
        f_v, ...
        r_v, ...
        ego_spd ];

    Nvalid = i;

    % ---- forward ----
    if i < Nx
        Vissim.Simulation.RunSingleStep;
    end
end

% ---- trim ----
Pred_speds = Pred_speds(1:Nvalid,:);

fprintf('\n')
fprintf('[VissimRollout] >> Prediction data logged\n');

end
