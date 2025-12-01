mu_Terre= 398600.4418;
Re = 6378.137;
% --- 2. Définition de l'État Initial (Exemple : LEO Circulaire) ---
altitude = 400; % km
r_magnitude = Re + altitude; % km

% Vitesse pour une orbite circulaire
v_magnitude = sqrt(mu_Terre / r_magnitude); % km/s
periode = 2 * pi * sqrt(r_magnitude^3 / mu_Terre); % Période (s)

% Vecteur d'état initial [x; y; z; vx; vy; vz]
etat = [
    r_magnitude; 
    0; 
    0; 
    0; 
    v_magnitude; 
    0
];
modele_dynamique(etat,mu_Terre);

TESTZVTZDVTVZD