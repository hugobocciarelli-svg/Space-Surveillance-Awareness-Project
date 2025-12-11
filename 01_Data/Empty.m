Extract_data(22)
function [x_eci, y_eci, z_eci] = aer_to_eci(azimuth_deg, elevation_deg, distance_m, ...
                                             lat_deg, lon_deg, alt_m, time_sec_j2000)

    % Conversion en radians
    az_rad = deg2rad(azimuth_deg);
    el_rad = deg2rad(elevation_deg);
    lat_rad = deg2rad(lat_deg);
    lon_rad = deg2rad(lon_deg);
    
    % Étape 1: AER vers coordonnées locales ENU (East-North-Up)
    e = distance_m .* cos(el_rad) .* sin(az_rad);
    n = distance_m .* cos(el_rad) .* cos(az_rad);
    u = distance_m .* sin(el_rad);
    
    % Étape 2: Position de l'observateur en ECEF (Earth-Centered Earth-Fixed)
    % Paramètres WGS84
    a = 6378137.0;  % rayon équatorial (m)
    f = 1/298.257223563;  % aplatissement
    e2 = 2*f - f^2;  % excentricité au carré
    
    N = a / sqrt(1 - e2 * sin(lat_rad)^2);
    
    obs_x_ecef = (N + alt_m) * cos(lat_rad) * cos(lon_rad);
    obs_y_ecef = (N + alt_m) * cos(lat_rad) * sin(lon_rad);
    obs_z_ecef = (N * (1 - e2) + alt_m) * sin(lat_rad);
    
    % Étape 3: Matrice de rotation ENU vers ECEF
    sin_lat = sin(lat_rad);
    cos_lat = cos(lat_rad);
    sin_lon = sin(lon_rad);
    cos_lon = cos(lon_rad);
    
    % Transformation ENU vers ECEF
    target_x_ecef = obs_x_ecef + (-sin_lon .* e - sin_lat * cos_lon .* n + cos_lat * cos_lon .* u);
    target_y_ecef = obs_y_ecef + (cos_lon .* e - sin_lat * sin_lon .* n + cos_lat * sin_lon .* u);
    target_z_ecef = obs_z_ecef + (cos_lat .* n + sin_lat .* u);
    
    % Étape 4: ECEF vers ECI (rotation due à la rotation de la Terre)
    % Vitesse angulaire de la Terre (rad/s)
    omega_earth = 7.2921159e-5;
    
    % Angle de rotation depuis J2000
    theta = omega_earth .* time_sec_j2000;
    
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    
    % Rotation autour de l'axe Z
    x_eci = target_x_ecef .* cos_theta - target_y_ecef .* sin_theta;
    y_eci = target_x_ecef .* sin_theta + target_y_ecef .* cos_theta;
    z_eci = target_z_ecef;
    
end