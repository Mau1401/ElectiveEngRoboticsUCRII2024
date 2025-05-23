J = [-0.05734547 -0.22030025 -0.16840403;
     0.65546168 -0.01927377 -0.01473344;
     0 0.65796544 0.36252311;
     0 0.08715574 0.08715574;
     0 -0.9961947 -0.9961947;
     1 0 0];

v_cart = [-0.02813 0.26173 0.02954 0.00000 0.00000 0.40000]';
q_dot = pinv(J) * v_cart;
% Recalcular la velocidad cartesiana alcanzada
v_cart_achieved = J * q_dot;

% Cálculo del error lineal y rotacional
error_linear = norm(v_cart(1:3) - v_cart_achieved(1:3));
error_rotational = norm(v_cart(4:6) - v_cart_achieved(4:6));

% Definición de tolerancia máxima (epsilon)
epsilon_linear = 0.01; % Tolerancia de velocidad lineal
epsilon_rotational = 0.01; % Tolerancia de velocidad rotacional

% Evaluación del cumplimiento de las velocidades deseadas
if error_linear <= epsilon_linear && error_rotational <= epsilon_rotational
    fprintf('El robot logra alcanzar la velocidad cartesiana deseada.\n');
else
    fprintf('El robot NO logra alcanzar la velocidad cartesiana deseada.\n');
end

% Resultados
fprintf('Velocidad en las articulaciones (q_dot):\n');
disp(q_dot);
fprintf('Error lineal: %.6f\n', error_linear);
fprintf('Error rotacional: %.6f\n', error_rotational);