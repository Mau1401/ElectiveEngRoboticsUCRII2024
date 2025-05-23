% Datos iniciales
Rw = 0.1015;
L = 0.6;
l = 0.7;
Ts = 0.01;
encoder_counts = [11833, 13209, 15926, 17974];
encoder_counts_prev = [11805, 13139, 15905, 17933];
conversion_factor = 2 * pi / 3415;
theta_k = pi / 4;
x_k = 0.7;
y_k = 0.9;

% Velocidad angular de las ruedas
delta_theta = (encoder_counts - encoder_counts_prev) * conversion_factor;
omega = delta_theta / Ts;

% Velocidad lineal de las ruedas
v_wheel = omega * Rw;

% Cálculo de v_x, v_y y omega_z
v_x = (v_wheel(1) + v_wheel(2) + v_wheel(3) + v_wheel(4)) / 4;
v_y = (-v_wheel(1) + v_wheel(2) - v_wheel(3) + v_wheel(4)) / 4;
omega_z = (-v_wheel(1) + v_wheel(2) - v_wheel(3) + v_wheel(4)) / (4 * (L + l));

% Actualización de la postura global
delta_x = (v_x * cos(theta_k) - v_y * sin(theta_k)) * Ts;
delta_y = (v_x * sin(theta_k) + v_y * cos(theta_k)) * Ts;
delta_theta = omega_z * Ts;

x_k1 = x_k + delta_x;
y_k1 = y_k + delta_y;
theta_k1 = theta_k + delta_theta;

% Mostrar resultados
fprintf('Velocidades angulares de las ruedas (rad/s): %.4f, %.4f, %.4f, %.4f\n', omega);
fprintf('Velocidades lineales de las ruedas (m/s): %.4f, %.4f, %.4f, %.4f\n', v_wheel);
fprintf('v_x: %.4f m/s, v_y: %.4f m/s, omega_z: %.4f rad/s\n', v_x, v_y, omega_z);
fprintf('Nueva postura (k+1): x = %.4f m, y = %.4f m, theta = %.4f rad\n', x_k1, y_k1, theta_k1);