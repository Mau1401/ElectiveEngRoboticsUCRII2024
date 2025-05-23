% Conversión de grados a radianes
deg_to_rad = pi / 180;

% Rotaciones en grados
theta_x1 = 33 * deg_to_rad;
theta_z = 56 * deg_to_rad;
theta_x2 = 45 * deg_to_rad;

% Matriz de rotación para 33° alrededor del eje x
Rx1 = [1, 0, 0;
       0, cos(theta_x1), -sin(theta_x1);
       0, sin(theta_x1), cos(theta_x1)];

% Matriz de rotación para 56° alrededor del eje z
Rz = [cos(theta_z), -sin(theta_z), 0;
      sin(theta_z), cos(theta_z), 0;
      0, 0, 1];

% Matriz de rotación para 45° alrededor del eje x
Rx2 = [1, 0, 0;
       0, cos(theta_x2), -sin(theta_x2);
       0, sin(theta_x2), cos(theta_x2)];

% Composición de las matrices de rotación
R = Rx2*Rz*Rx1;

% Mostrar el resultado
fprintf('La matriz de rotación compuesta es:\n');
fprintf('[%.4f, %.4f, %.4f;\n %.4f, %.4f, %.4f;\n %.4f, %.4f, %.4f]\n', ...
        R(1,1), R(1,2), R(1,3), ...
        R(2,1), R(2,2), R(2,3), ...
        R(3,1), R(3,2), R(3,3));
