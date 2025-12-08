%% MAIN - Filtre de Kalman pour un satellite unique
% Nettoyage
clear; clc; close all;

%% 1. PARAMÈTRES DE SIMULATION
t0 = 0;           % Temps initial [s]
dt = 50;          % Pas de temps [s]
t_end = 24000;    % Temps final [s] (~6.67 heures)
TimeT = t0:dt:t_end; % Vecteur temps
n_steps = length(TimeT); % Nombre de pas

%% 2. CONDITIONS INITIALES
% État initial: [x, y, z, vx, vy, vz] en km et km/s
X0_true = [7078; 0; 0; 0; 7.5; 0]; % Vérité terrain
X0_est = X0_true + [10; 5; -5; 0.1; -0.1; 0.05]; % Estimation initiale (avec erreur)

%% 3. PARAMÈTRES DU FILTRE DE KALMAN
% Matrice de mesure (on mesure seulement la position)
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

% Covariance initiale de l'estimation
P_est = diag([100, 100, 100, 0.1, 0.1, 0.1]); % Grande incertitude initiale

% Bruit du processus (incertitude du modèle)
Q = diag([0.001, 0.001, 0.001, 0.00001, 0.00001, 0.00001]);

% Bruit de mesure (incertitude des capteurs)
R = diag([25, 25, 25]); % Variance de 25 km² sur chaque position

%% 4. INITIALISATION DES VARIABLES DE STOCKAGE
% Pour la vérité terrain
true_states = zeros(6, n_steps);
true_states(:, 1) = X0_true;

% Pour l'estimation
estimated_states = zeros(6, n_steps);
estimated_states(:, 1) = X0_est;

% Pour les prédictions (avant correction)
predicted_states = zeros(6, n_steps);
predicted_states(:, 1) = X0_est;

% Pour les mesures
measurements = zeros(3, n_steps);

% Pour la covariance
P_history = cell(1, n_steps);
P_history{1} = P_est;

%% 5. BOUCLE PRINCIPALE - SIMULATION ET FILTRAGE
for i = 1:n_steps-1
    t = TimeT(i);
    
    % ========== SIMULATION DE LA VÉRITÉ TERRAIN ==========
    % Intégration de l'état vrai
    true_states(:, i+1) = RungeKutta4(@modele_dynamique, t, true_states(:, i), dt);
    
    % ========== GÉNÉRATION DE LA MESURE ==========
    [measurement, true_pos] = generate_measurement(true_states(:, i+1), R);
    measurements(:, i+1) = measurement;
    
    % ========== ÉTAPE DE PRÉDICTION DU FILTRE ==========
    % Prédiction de l'état (modèle dynamique)
    x_pred = RungeKutta4(@modele_dynamique, t, estimated_states(:, i), dt);
    predicted_states(:, i+1) = x_pred;
    
    % Prédiction de la covariance (simplifiée)
    P_pred = P_est + Q;
    
    % ========== ÉTAPE DE CORRECTION DU FILTRE ==========
    % Mise à jour avec la mesure
    [x_updated, P_updated] = kalman_filter_step(x_pred, P_pred, measurement, H, R);
    
    % Stockage des résultats
    estimated_states(:, i+1) = x_updated;
    P_est = P_updated;
    P_history{i+1} = P_updated;
end

%% 6. VISUALISATION DES RÉSULTATS
% ========== GRAPHIQUE 1: Trajectoires 2D ==========
figure('Position', [100, 100, 1200, 500]);

% Sous-figure 1: Plan XY
subplot(1,3,1);
hold on; grid on;
plot(true_states(1,:), true_states(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Vérité terrain');
plot(predicted_states(1,:), predicted_states(2,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Prédiction');
plot(estimated_states(1,:), estimated_states(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimation');
scatter(measurements(1,2:end), measurements(2,2:end), 10, 'g', 'filled', 'DisplayName', 'Mesures');
xlabel('X (km)'); ylabel('Y (km)');
title('Plan XY');
legend('Location', 'best');
axis equal;

% Sous-figure 2: Plan XZ
subplot(1,3,2);
hold on; grid on;
plot(true_states(1,:), true_states(3,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Vérité terrain');
plot(predicted_states(1,:), predicted_states(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Prédiction');
plot(estimated_states(1,:), estimated_states(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimation');
scatter(measurements(1,2:end), measurements(3,2:end), 10, 'g', 'filled', 'DisplayName', 'Mesures');
xlabel('X (km)'); ylabel('Z (km)');
title('Plan XZ');
legend('Location', 'best');
axis equal;

% Sous-figure 3: Plan YZ
subplot(1,3,3);
hold on; grid on;
plot(true_states(2,:), true_states(3,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Vérité terrain');
plot(predicted_states(2,:), predicted_states(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Prédiction');
plot(estimated_states(2,:), estimated_states(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimation');
scatter(measurements(2,2:end), measurements(3,2:end), 10, 'g', 'filled', 'DisplayName', 'Mesures');
xlabel('Y (km)'); ylabel('Z (km)');
title('Plan YZ');
legend('Location', 'best');
axis equal;

sgtitle('Comparaison des trajectoires - Vue 2D');

% ========== GRAPHIQUE 2: Erreurs d'estimation ==========
figure('Position', [100, 100, 1000, 600]);

% Calcul des erreurs
pos_error = sqrt(sum((estimated_states(1:3,:) - true_states(1:3,:)).^2, 1));
vel_error = sqrt(sum((estimated_states(4:6,:) - true_states(4:6,:)).^2, 1));

% Erreur de position
subplot(2,1,1);
plot(TimeT, pos_error, 'b-', 'LineWidth', 2);
grid on; hold on;
xlabel('Temps (s)'); ylabel('Erreur (km)');
title('Erreur d''estimation de position');
legend('RMSE Position');

% Erreur de vitesse
subplot(2,1,2);
plot(TimeT, vel_error, 'r-', 'LineWidth', 2);
grid on;
xlabel('Temps (s)'); ylabel('Erreur (km/s)');
title('Erreur d''estimation de vitesse');
legend('RMSE Vitesse');

% ========== GRAPHIQUE 3: Trajectoire 3D ==========
trace_orbite(true_states); % Vérité terrain en noir
hold on;
plot3(estimated_states(1,:), estimated_states(2,:), estimated_states(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimation');
legend('Terre', 'Vérité terrain', 'Estimation');

%% 7. AFFICHAGE DES STATISTIQUES
fprintf('\n========== STATISTIQUES DU FILTRE ==========\n');
fprintf('Erreur moyenne de position: %.2f km\n', mean(pos_error(100:end))); % Ignorer les 100 premiers points
fprintf('Erreur moyenne de vitesse: %.4f km/s\n', mean(vel_error(100:end)));
fprintf('Erreur max de position: %.2f km\n', max(pos_error));
fprintf('Erreur max de vitesse: %.4f km/s\n', max(vel_error));

%% 8. PRÉDICTION FUTURE (au-delà des mesures)
fprintf('\n========== PRÉDICTION FUTURE ==========\n');
% Temps de prédiction supplémentaire
t_pred_start = t_end;
t_pred_end = t_end + 3600; % +1 heure
dt_pred = 100; % Pas plus grand pour la prédiction
n_pred_steps = ceil((t_pred_end - t_pred_start) / dt_pred);

% Initialiser à partir de la dernière estimation
x_pred_future = estimated_states(:, end);
pred_future = zeros(6, n_pred_steps);

for i = 1:n_pred_steps
    % Prédiction simple (sans correction)
    x_pred_future = RungeKutta4(@modele_dynamique, t_pred_start + (i-1)*dt_pred, x_pred_future, dt_pred);
    pred_future(:, i) = x_pred_future;
end

% Visualisation de la prédiction future
figure;
hold on; grid on;
plot3(true_states(1,:), true_states(2,:), true_states(3,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Vérité terrain (mesurée)');
plot3(estimated_states(1,:), estimated_states(2,:), estimated_states(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimation');
plot3(pred_future(1,:), pred_future(2,:), pred_future(3,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Prédiction future');
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Prédiction de trajectoire future');
legend('Location', 'best');
axis equal;
view(45, 25);

fprintf('Prédiction future calculée sur %.1f heures supplémentaires\n', (t_pred_end - t_pred_start)/3600);