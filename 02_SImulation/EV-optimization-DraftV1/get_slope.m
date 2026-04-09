% function result = get_slope(deg_fit, N_dist, start)
% x = (N_dist+start-35.37)/28.23;
% result = deg_fit(1)*x^4 + ...
%          deg_fit(2)*x^3 + ...
%          deg_fit(3)*x^2 + ...
%          deg_fit(4)*x^1 + ...
%          deg_fit(5);

function grade = get_slope(dist_m, degmap, start)
% dist_m: 누적거리 [m]
% degmap: [distance, slope(grade), elevation] from CSV
% start: offset [m]

x = dist_m + start; 

grade = interp1(degmap(:,1), degmap(:,2), x, 'linear', 'extrap');
grade = max(min(grade, 0.3), -0.3);
end