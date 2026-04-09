function LaneInfo = get_LaneInfo(Vissim, egoPos, egoLen, TargetID, link_num, LaneIdxList, NO_CAR_DIST)
% LaneInfo(k): lane, f_dist, f_v, r_dist, r_v
% dist는 bumper-to-bumper gap [m]
% - front gap: (front car rear) - (ego front) = (veh_pos - veh_len) - egoPos
% - rear  gap: (ego rear) - (rear car front) = (egoPos - egoLen) - veh_pos

if nargin < 7, NO_CAR_DIST = 50; end

All_Vehicles = Vissim.Net.Vehicles.GetAll;

LaneIdxList = LaneIdxList(:)'; % row
LaneInfo = repmat(struct('lane',NaN,'f_dist',NO_CAR_DIST,'f_v',0,'r_dist',NO_CAR_DIST,'r_v',0), ...
                  1, numel(LaneIdxList));

for k = 1:numel(LaneIdxList)
    ln = LaneIdxList(k);

    f_dist = inf; f_v = 0;
    r_dist = inf; r_v = 0;

    for i = 1:numel(All_Vehicles)
        veh = All_Vehicles{i};

        try
            v_no = double(get(veh,'AttValue','No'));
            if v_no == TargetID, continue; end

            veh_lane_str = char(get(veh,'AttValue','Lane'));
            if isempty(veh_lane_str) || ~contains(veh_lane_str,'-')
                continue;
            end

            veh_link = str2double(extractBefore(veh_lane_str,'-'));
            veh_lane = str2double(extractAfter(veh_lane_str,'-'));
            if ~isfinite(veh_link) || ~isfinite(veh_lane), continue; end

            if veh_lane ~= ln, continue; end
            if veh_link ~= link_num, continue; end

            veh_pos = double(get(veh,'AttValue','Pos'));        % assume front bumper [m]
            veh_spd = double(get(veh,'AttValue','Speed'))/3.6;  % m/s
            veh_len = double(get(veh,'AttValue','Length'));     % m

            % -------------------------
            % bumper-to-bumper gaps (FIX)
            % -------------------------
            if veh_pos > egoPos
                % front vehicle
                dist = (veh_pos - veh_len) - egoPos;           % (front rear) - (ego front)
                if dist < f_dist
                    f_dist = dist;
                    f_v = veh_spd;
                end
            elseif veh_pos < egoPos
                % rear vehicle
                dist = (egoPos - egoLen) - veh_pos;            % (ego rear) - (rear front)
                if dist < r_dist
                    r_dist = dist;
                    r_v = veh_spd;
                end
            end
        catch
            continue;
        end
    end

    if ~isfinite(f_dist), f_dist = NO_CAR_DIST; f_v = 0; end
    if ~isfinite(r_dist), r_dist = NO_CAR_DIST; r_v = 0; end

    LaneInfo(k).lane   = ln;
    LaneInfo(k).f_dist = max(0, f_dist);
    LaneInfo(k).f_v    = f_v;
    LaneInfo(k).r_dist = max(0, r_dist);
    LaneInfo(k).r_v    = r_v;
end
end
