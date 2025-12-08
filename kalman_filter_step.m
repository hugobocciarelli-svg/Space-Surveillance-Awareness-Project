function [x_updated, P_updated] = kalman_filter_step(x_pred, P_pred, z_measurement, H, R)
% Une étape du filtre de Kalman (version linéaire simple)
% Inputs:
%   x_pred: état prédit [6x1]
%   P_pred: covariance prédite [6x6]
%   z_measurement: mesure [3x1]
%   H: matrice de mesure [3x6]
%   R: covariance du bruit de mesure [3x3]
% Outputs:
%   x_updated: état mis à jour
%   P_updated: covariance mise à jour

    % Innovation (erreur entre mesure et prédiction)
    y = z_measurement - H * x_pred;
    
    % Covariance de l'innovation
    S = H * P_pred * H' + R;
    
    % Gain de Kalman
    K = P_pred * H' / S;
    
    % Mise à jour de l'état
    x_updated = x_pred + K * y;
    
    % Mise à jour de la covariance (forme simple)
    P_updated = (eye(6) - K * H) * P_pred;
end