function ECI = ECEFtoECI(ECEF,date)
n_meas = size(ECEF, 2); % Nombre de mesures (N)
ECI = zeros(3, n_meas); % Initialisation 
    for k = 1:n_meas
        r_ecef_k = ECEF(:, k); 
        utc_datetime_k = date(k); 
        [r_eci_k, ~] = ecef2eci(utc_datetime_k, r_ecef_k); 
        ECI(:, k) = r_eci_k;
    end
end
