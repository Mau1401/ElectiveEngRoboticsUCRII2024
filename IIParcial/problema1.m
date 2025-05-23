% Datos iniciales
v = 1.9; % Velocidad lineal [m/s]
x_r = 3.7; y_r = 6.3; theta_r = 1.1; % Coordenadas actuales
x_ob = 4.7; y_ob = 7.7; % Coordenadas objetivo
b = 0.2727; % Distancia entre ruedas [m]

% Cálculo de la distancia al objetivo (LA)
L_A = sqrt((x_ob - x_r)^2 + (y_ob - y_r)^2);

% Cálculo del desplazamiento lateral (Delta x)
Delta_x = -(x_ob - x_r) * sin(theta_r) + (y_ob - y_r) * cos(theta_r);

% Factor de corrección de velocidad (gamma)
gamma = 2 * Delta_x / (L_A^2);

% Velocidades de las ruedas
v_d = v + (gamma * v * b) / 2; % Velocidad de la rueda derecha
v_i = v - (gamma * v * b) / 2; % Velocidad de la rueda izquierda

% Resultados
fprintf('Distancia al objetivo (L_A): %.4f m\n', L_A);
fprintf('Desplazamiento lateral (Delta x): %.4f m\n', Delta_x);
fprintf('Factor de corrección de velocidad (gamma): %.4f\n', gamma);
fprintf('Velocidad rueda derecha (v_d): %.4f m/s\n', v_d);
fprintf('Velocidad rueda izquierda (v_i): %.4f m/s\n', v_i);
