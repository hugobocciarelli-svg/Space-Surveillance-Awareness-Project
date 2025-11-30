function d_etat = modele_dynamique(etat,mu)
%MODELE_DYNAMIQUE_2CORPS Calcule la dérivée d'état pour le problème à deux corps.
    r_vec = etat(1:3); % Vecteur position [x; y; z]
    v_vec = etat(4:6); % Vecteur vitesse [vx; vy; vz]
    r = norm(r_vec);
    
    if r == 0
        a_gravite = [0; 0; 0];
    else
        a_gravite = -(mu / r^3) * r_vec;
    end
    d_etat = [
        v_vec;    % d/dt(r) = v
        a_gravite % d/dt(v) = a
    ];
end