function [distance, time_of_closest_approach] = distance_relative(traj1, traj2, TimeT)
% Calcule la distance relative entre deux satellites
% Inputs:
%   traj1, traj2: matrices [6 x n] des états des satellites
%   TimeT: vecteur temps [1 x n]
% Outputs:
%   distance: vecteur des distances [1 x n]
%   time_of_closest_approach: temps du rapprochement minimal

    % Extraction des positions
    x1 = traj1(1, :);
    y1 = traj1(2, :);
    z1 = traj1(3, :);
    
    x2 = traj2(1, :);
    y2 = traj2(2, :);
    z2 = traj2(3, :);
    
    % Calcul de la distance
    distance = sqrt((x2 - x1).^2 + (y2 - y1).^2 + (z2 - z1).^2);
    
    % Trouver le temps du rapprochement minimal
    [min_dist, idx_min] = min(distance);
    time_of_closest_approach = TimeT(idx_min);
    
    fprintf('Distance minimale: %.2f km à t = %.1f s\n', min_dist, time_of_closest_approach);
end
