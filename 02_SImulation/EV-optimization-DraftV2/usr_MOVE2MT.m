function dyn_Torque = usr_MOVE2MT(ACC,SP)

m = 1685 + 65;
g = 9.81; % m/s^2
RRC = (0.0066+0.0077)/2; % dry condition
R = 0.33415;% Tire radius
gear = 7.981;% Gear ratio

dyn_Torque = (m.*ACC+1/2*0.5*1.188*2.06*(SP/3.6).^2+...
              RRC*m*g)*R/gear;

end