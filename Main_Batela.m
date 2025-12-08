clear; clc; close all;

Target_selected = 22 ; %id de la target

%% 1.Extraction des mesures et conversion en ECI
meas = Extract_data(Target_selected);
Z = [meas.x_true'; meas.y_true'; meas.z_true']; %Mise en forme du vecteur de mesure Z
n_meas = size(Z, 2);            %Nombre de mesures prises en compte
Z_eci = zeros(3, n_meas);       %Initialisation du vecteur Z_ECI
for k = 1:n_meas                %Conversion de chaque données du vecteur de ECEF evrs ECI
    r_ecef_k = Z(:, k); 
    utc_datetime_k = meas.datetime(k); 
    [r_eci_k, ~] = ecef2eci(utc_datetime_k, r_ecef_k); 
    Z_eci(:, k) = r_eci_k;
end

idx_meas=1 ; %index de la mesure actuelle

%% 2. PARAMÈTRES DE SIMULATION
t0 = meas.time(1);                  % Temps initial [s]
dt = 1.6;                           % Pas de temps [s](Synchro sur le dt mesure pour des questions de simplicités)
t_sim = 20;                      % Temps de simulation [s] 
t_end = t0 + t_sim;                 % Temps  final [s] 
TimeT = t0:dt:t_end;                % Vecteur temps
n_steps = length(TimeT);            % Nombre de pas

%% 3. CONDITIONS INITIALES
% État initial: [x, y, z, vx, vy, vz] en km et km/s
X0_real = [7078; 0; 0; 0; 7.5; 0];                  % Supposons que nous connaissons l'etat initial
X0_est = X0_real + [10; 5; -5; 0.1; -0.1; 0.05];    % Estimation initiale (avec erreur)

%% 4. PARAMÈTRES DU FILTRE DE KALMAN
% Matrice de mesure 
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

% Covariance initiale de l'estimation
P_est = diag([100, 100, 100, 0.1, 0.1, 0.1]); 

% Bruit du processus (incertitude du modèle)
W = diag([0.001, 0.001, 0.001, 0.00001, 0.00001, 0.00001]);

% Bruit de mesure (incertitude des capteurs)
V = diag([2, 2, 2]); % Variance de 25 km² sur chaque position

%% 5. INITIALISATION DES VARIABLES DE STOCKAGE
% Pour les vecteurs corrigés (+ initial)
Corrected_states = zeros(6, n_steps);
Corrected_states(:, 1) = X0_est;

% Pour les prédictions (avant correction)
predicted_states = zeros(6, n_steps);
predicted_states(:, 1) = X0_est;

% Pour la covariance
P_history = cell(1, n_steps);
P_history{1} = P_est;  
compteur_corr=0;

%% 6. BOUCLE PRINCIPALE - SIMULATION ET FILTRAGE
for i = 1:n_steps-1
    t = TimeT(i);
    
    %Prédiction du vecteur d'etat 
    x_pred = RungeKutta4(@modele_dynamique, t, Corrected_states(:, i), dt);
    predicted_states(:, i+1) = x_pred;

    % Matrice de transition d'état (Jacobienne)
    F=MatriceTransition(x_pred,dt);

    %Propagation de la covariance
    P_pred = F * P_history{i} * F' + W;
    
    % Cherche si une mesure existe proche du temps actuel (tolérance de dt/2)
    measure_dispo = false;
    if idx_meas <= n_meas
        time_diff = abs(meas.time(idx_meas) - t);
        if time_diff < dt/2  % Tolérance
            measure_dispo = true;
        end
    end
    
    if measure_dispo
        compteur_corr=compteur_corr+1;
        % extraction de la mesure actuelle 
        z_k = Z_eci(:, idx_meas);
        % Mise à jour de Kalman
        [X, P] = KalmanFilter(x_pred, P_pred, z_k, H, V);
        % Stocker les résultats corrigés
        Corrected_states(:, i+1) = X;
        P_history{i+1} = P;
        
        % Passer à la mesure suivante
        idx_meas = idx_meas + 1;
        
    else     %Si pas de mesure disponible on fait une estimation sans correction
        Corrected_states(:, i+1) = x_pred;
        P_history{i+1} = P_pred;
    end
end
delta = predicted_states-Corrected_states;


%% 7. Affichage
Plots(TimeT,predicted_states,Corrected_states)
