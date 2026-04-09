function target_num = detect_target(All_Vehicles,Target)
%%
len = length(All_Vehicles);


for cnt_Veh = 1 : len
    
    if(get(All_Vehicles{cnt_Veh}, 'AttValue', 'No') == Target)
        target_num = cnt_Veh;  
        break
    end
end


end