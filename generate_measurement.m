function [measurement, true_position] = generate_measurement(true_state, R_noise)
% Génère une mesure bruitée à partir d'un état vrai
% Inputs:
%   true_state: état vrai [6x1]
%   R_noise: matrice de covariance du bruit de mesure [3x3]
% Outputs:
%   measurement: mesure bruitée [3x1] (position seulement)
%   true_position: position vraie [3x1]

    % Position vraie
    true_position = true_state(1:3);
    
    % Génération du bruit gaussien
    noise = chol(R_noise)' * randn(3, 1);
    
    % Mesure bruitée
    measurement = true_position + noise;
end