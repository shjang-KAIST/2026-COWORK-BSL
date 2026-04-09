function get_TargetVehicle(Vissim, TargetID, halfBox_m)
% track_target

if nargin < 3 || isempty(halfBox_m)
    halfBox_m = 50;
end

% 1) get target coord (CoordFront: 'X Y Z')
try
    veh = Vissim.Net.Vehicles.ItemByKey(int32(TargetID));
    if isempty(veh), return; end
    s = get(veh,'AttValue','CoordFront'); % 'X Y Z'
catch
    return;
end

xyz = sscanf(char(s),'%f');
if numel(xyz) < 2, return; end
x = xyz(1);  y = xyz(2);

% 2) network window handle
try
    win = Vissim.Graphics.CurrentNetworkWindow;
catch
    return;
end
if isempty(win), return; end

% 3) bounding box
xmin = x - halfBox_m; xmax = x + halfBox_m;
ymin = y - halfBox_m; ymax = y + halfBox_m;

% 4) zoom
try
    invoke(win, 'ZoomTo', xmin, ymin, xmax, ymax);
catch
    try
        win.ZoomTo(xmin, ymin, xmax, ymax);
    catch
    end
end

end
