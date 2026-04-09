function [f_spacing,lead_speddiff, r_spacing, follow_speddiff] = ...
    detect_spacing(All_Vehicles,target_veh)
%%
len = length(All_Vehicles);
veh_list = zeros(len,4);

for cnt_Veh = 1 : len
    
    veh_list(cnt_Veh,1) = get(All_Vehicles{cnt_Veh}, 'AttValue', 'Speed');
    veh_list(cnt_Veh,2) = get(All_Vehicles{cnt_Veh}, 'AttValue', 'Pos');
    veh_linklane = get(All_Vehicles{cnt_Veh}, 'AttValue', 'Lane');
    
    veh_list(cnt_Veh,3) = str2double(veh_linklane(1));
    veh_list(cnt_Veh,4) = str2double(veh_linklane(3));   
end
%tar_link = target_veh(3);

same_links = veh_list(:,3) == target_veh(3);
same_lanes = veh_list(:,4) == target_veh(4);
kernel = same_links & same_lanes;

sped_list = veh_list(:,1);
pos_list = veh_list(:,2);

sped = sped_list(kernel);
pos = pos_list(kernel);

%%% the spacing between target and the front vehicle
logic_pos = pos > target_veh(2);
cand_pos = pos(logic_pos);
leading_pos = min(cand_pos);
if(sum(cand_pos) ~= 0)
    f_spacing = leading_pos - target_veh(2);
    %%% the speed difference between target and the front vehicle
    [i, ] = find(pos == leading_pos);
    lead_speddiff = sped(i) - target_veh(1);
else
    f_spacing = 100;
    lead_speddiff = 0;
end

%%% the spacing between target and the rear vehicle
logic_pos = pos < target_veh(2);
cand_pos = pos(logic_pos);
follow_pos = max(cand_pos);
if(sum(cand_pos) ~= 0)
    r_spacing = target_veh(2) - follow_pos;
    %%% the speed difference between target and the front vehicle
    [j, ] = find(pos == follow_pos);
    follow_speddiff = target_veh(1) - sped(j);
else
    r_spacing = 100;
    follow_speddiff = 0;
end


end