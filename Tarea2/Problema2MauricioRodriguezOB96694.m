%IE1103 Temas Especiales II en Ingeniería
%Problema 2 Tarea 2
%Mauricio Rodirguez Obando, B96694
% Definir las variables simbólicas
syms theta1 theta2 theta3

% Definir las matrices de transformación intermedias
A1 = [cos(theta1), 0, sin(theta1), 0;
      sin(theta1), 0, -cos(theta1), 0;
      0, 1, 0, 0;
      0, 0, 0, 1];

A2 = [cos(theta2), -sin(theta2), 0, 0.3*cos(theta2);
      sin(theta2), cos(theta2), 0, 0.3*sin(theta2);
      0, 0, 1, 0.1;
      0, 0, 0, 1];

A3 = [cos(theta3), -sin(theta3), 0, 0.4*cos(theta3);
      sin(theta3), cos(theta3), 0, 0.4*sin(theta3);
      0, 0, 1, 0;
      0, 0, 0, 1];

% Calcular la matriz de transformación total T03
T03_unsimp = A1 * A2 * A3;
%Simplificando
T03 = simplify(T03_unsimp);

% Mostrar el resultado
disp('La matriz de transformación T03 es:');
disp(T03_unsimp);
disp('Y simplificada:');
disp(T03);
