function F = calcul_STM(x, dt)
    r = x(1:3);
    r_norm = norm(r);
    mu = 398600.4418;  
    
    % Jacobien de l'accélération gravitationnelle
    A = -mu/r_norm^3 * (eye(3) - 3*(r*r')/(r_norm^2));
    
    % Matrice F continue
    F_cont = [zeros(3), eye(3);
              A,        zeros(3)];
    
    % Discrétisation (approximation)
    F = eye(6) + dt * F_cont;
end