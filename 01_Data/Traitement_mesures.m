% Script de traitement de données et formatage des données d'entrées
% Auteur: Hugo Bocciarelli

loaded_data = load('mesures.mat');
mesures = loaded_data.mesures;
%1.PARAMÈTRES DU CAPTEUR
sensor = struct();
sensor.latitude = 45.0;      % deg
sensor.longitude = 0.0;      % deg
sensor.altitude = 0.0;       % m
sensor.position = [0, 0, 0]; % Position cartésienne (à calculer si besoin)

%2. PARAMÈTRES DES ERREURS
errors = struct();
errors.azimuth_std = 0.1;    % deg
errors.elevation_std = 0.1;  % deg
errors.distance_std = 50;    % m
errors.correlation = false;  % Non-corrélées


%3. ORGANISATION DE LA STRUCTURE DES MESURES
measurements = struct();

% Extraction des colonnes
measurements.target_id = mesures(:, 1);
measurements.time = mesures(:, 2);  % secondes depuis 01/01/2000 00:00:00

% Coordonnées cartésiennes géocentriques (valeurs théoriques)
measurements.x_true = mesures(:, 3);  % m
measurements.y_true = mesures(:, 4);  % m
measurements.z_true = mesures(:, 5);  % m

% Mesures sphériques (observateur)
measurements.azimuth = mesures(:, 6);    % deg, [-180, 180], 0=Nord
measurements.elevation = mesures(:, 7);  % deg, [0, 90], 0=horizon
measurements.distance = mesures(:, 8);   % m

% Nombre de mesures
measurements.n_measurements = size(mesures, 1);

% Identifiants uniques des cibles
measurements.unique_targets = unique(measurements.target_id);
measurements.n_targets = length(measurements.unique_targets);

%% 4. CONVERSION TEMPS EN FORMAT LISIBLE
% Référence: 01/01/2000 00:00:00
epoch_2000 = datetime(2000, 1, 1, 0, 0, 0);
measurements.datetime = epoch_2000 + seconds(measurements.time);

%% 5. CALCUL DE LA POSITION VRAIE EN COORDONNÉES SPHÉRIQUES
measurements.range_true = sqrt(measurements.x_true.^2 + ...
                               measurements.y_true.^2 + ...
                               measurements.z_true.^2);

%% 6. AFFICHAGE RÉSUMÉ
fprintf('\n=== RÉSUMÉ DES DONNÉES ===\n');
fprintf('Nombre de mesures: %d\n', measurements.n_measurements);
fprintf('Nombre de cibles: %d\n', measurements.n_targets);
fprintf('Période: %s à %s\n', ...
        datestr(measurements.datetime(1)), ...
        datestr(measurements.datetime(end)));
fprintf('Plage azimuth: [%.2f, %.2f] deg\n', ...
        min(measurements.azimuth), max(measurements.azimuth));
fprintf('Plage élévation: [%.2f, %.2f] deg\n', ...
        min(measurements.elevation), max(measurements.elevation));
fprintf('Plage distance: [%.2f, %.2f] m\n', ...
        min(measurements.distance), max(measurements.distance));

%% 7. ORGANISATION PAR CIBLE 
targets = struct();
for i = 1:measurements.n_targets
    target_id = measurements.unique_targets(i);
    idx = measurements.target_id == target_id;
    
    targets(i).id = target_id;
    targets(i).time = measurements.time(idx);
    targets(i).datetime = measurements.datetime(idx);
    targets(i).x_true = measurements.x_true(idx);
    targets(i).y_true = measurements.y_true(idx);
    targets(i).z_true = measurements.z_true(idx);
    targets(i).azimuth = measurements.azimuth(idx);
    targets(i).elevation = measurements.elevation(idx);
    targets(i).distance = measurements.distance(idx);
    targets(i).n_measurements = sum(idx);
end

fprintf('\n=== STRUCTURE CRÉÉE ===\n');
fprintf('Variables disponibles:\n');
fprintf('  - sensor: paramètres du capteur\n');
fprintf('  - errors: parametres d''erreur\n');
fprintf('  - measurements: toutes les mesures organisées\n');
fprintf('  - targets: mesures organisées par cible\n');

