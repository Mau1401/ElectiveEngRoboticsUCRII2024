% Parámetros del robot
a1 = 0.5; a2 = 0.4; a3 = 0.3;

% Ángulos en radianes
q = [35, 55, -15] * pi/180;
q_dot = [10, 40, -10] * pi/180; % Velocidades angulares en rad/s

% Cálculo de senos y cosenos
s1 = sin(q(1)); c1 = cos(q(1));
s12 = sin(q(1) + q(2)); c12 = cos(q(1) + q(2));
s123 = sin(q(1) + q(2) + q(3)); c123 = cos(q(1) + q(2) + q(3));

% Jacobiano
J = [-a1*s1 - a2*s12 - a3*s123, -a2*s12 - a3*s123, -a3*s123;
      a1*c1 + a2*c12 + a3*c123,  a2*c12 + a3*c123,  a3*c123;
      1, 1, 1];

% Parte A: Mostrar el Jacobiano
disp('Jacobiano:');
disp(J);

% Parte B: Cálculo de la velocidad del efector final
vel_efector = J * q_dot';
disp('Velocidad del efector final:');
disp(vel_efector);
