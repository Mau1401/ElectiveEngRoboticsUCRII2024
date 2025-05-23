%IE1103 Temas Especiales II en Ingeniería
%Problema 3 Tarea 2
%Mauricio Rodirguez Obando, B96694

% Definir el vector original
z = [0.1, 0.3, 0.3]';

% Rotación a) 30° alrededor del eje z
theta = 30 * pi / 180; % Convertir a radianes
Rz = [cos(theta), -sin(theta), 0;
      sin(theta), cos(theta), 0;
      0, 0, 1];
z_rot_a = Rz * z;
disp('Rotacion de 30 en z del vector dado:')
disp(z_rot_a);

% Rotación b) con la matriz dada
R_given = [0.925417, 0.0180283, 0.378522;
           0.163176, 0.882564, 0.44097;
           -0.34202, 0.469846, 0.813798];
z_rot_b = R_given * z;
disp('Rotacion de acuerdo con matriz de rotacion:')
disp(z_rot_b);

% Rotación c) "roll, pitch, yaw" (10°, 20°, 30°)
roll = 10 * pi / 180;
pitch = 20 * pi / 180;
yaw = 30 * pi / 180;

Rx = [1, 0, 0;
      0, cos(roll), -sin(roll);
      0, sin(roll), cos(roll)];
Ry = [cos(pitch), 0, sin(pitch);
      0, 1, 0;
      -sin(pitch), 0, cos(pitch)];
Rz = [cos(yaw), -sin(yaw), 0;
      sin(yaw), cos(yaw), 0;
      0, 0, 1];

R_total = Rz * Ry * Rx;
disp('Rotacion Total:')
disp(R_total);

z_rot_c = R_total * z;
disp('Rotacion de acuerdo con roll, pitch y yaw:')
disp(z_rot_c);

% Graficar los resultados
figure;
quiver3(0, 0, 0, z(1), z(2), z(3), 'k', 'LineWidth', 2); hold on;
quiver3(0, 0, 0, z_rot_a(1), z_rot_a(2), z_rot_a(3), 'r', 'LineWidth', 2);
quiver3(0, 0, 0, z_rot_b(1), z_rot_b(2), z_rot_b(3), 'g', 'LineWidth', 2);
quiver3(0, 0, 0, z_rot_c(1), z_rot_c(2), z_rot_c(3), 'b', 'LineWidth', 2);

% Configuración del gráfico
legend('Original', 'Rotación a)', 'Rotación b)', 'Rotación c)');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Rotaciones de vector en 3D');
grid on;
axis equal;