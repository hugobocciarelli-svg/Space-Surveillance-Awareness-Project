function [collision_detected, collision_time, risk_level] = detect_collision(distance, TimeT, seuil_collision)
% Détection de collision avec évaluation du risque
% Inputs:
%   distance: vecteur des distances [1 x n]
%   TimeT: vecteur des temps [1 x n]
%   seuil_collision: seuil de distance critique (km)
% Outputs:
%   collision_detected: booléen
%   collision_time: temps de la collision potentielle
%   risk_level: niveau de risque (1=élevé, 2=modéré, 3=faible)

    % Trouver les indices où la distance est sous le seuil
    idx_risk = find(distance < seuil_collision);
    
    if isempty(idx_risk)
        collision_detected = false;
        collision_time = NaN;
        risk_level = 0;
        fprintf('Aucun risque de collision détecté\n');
        return;
    end
    
    collision_detected = true;
    collision_time = TimeT(idx_risk(1));
    
    % Calcul du niveau de risque
    min_distance = min(distance);
    if min_distance < seuil_collision * 0.1
        risk_level = 1; % Risque élevé
        warning_msg = 'RISQUE ÉLEVÉ';
    elseif min_distance < seuil_collision * 0.5
        risk_level = 2; % Risque modéré
        warning_msg = 'Risque modéré';
    else
        risk_level = 3; % Risque faible
        warning_msg = 'Risque faible';
    end
    
    % Affichage des résultats
    fprintf('\n========== DÉTECTION DE COLLISION ==========\n');
    fprintf('Collision potentielle détectée!\n');
    fprintf('Temps de la première approche critique: %.1f s\n', collision_time);
    fprintf('Distance minimale: %.2f km (seuil: %.2f km)\n', min_distance, seuil_collision);
    fprintf('Niveau de risque: %s\n', warning_msg);
    fprintf('Nombre de points à risque: %d\n', length(idx_risk));
end