function grade = get_slope(dist_m, degmap, start)
% dist_m: distance [m]
% degmap: [distance, grade] 
% start : offset [m]

x = dist_m + start;

if isempty(degmap) || size(degmap,2) < 2
    grade = 0;
    return;
end

% interpolate
grade = interp1(degmap(:,1), degmap(:,2), x, 'linear', 'extrap');

% clamp
grade = max(min(grade, 0.3), -0.3);
end
