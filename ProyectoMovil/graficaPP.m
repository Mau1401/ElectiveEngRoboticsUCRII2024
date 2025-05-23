%Carga de trayectorias
dataRefSquare = readmatrix("Square.csv");

% Extraer las columnas de coordenadas 
xRef = dataRefSquare(:,2);
yRef = dataRefSquare(:,3);

% Cargar datos de las simulaciones
% Asegúrate de tener los archivos CSV en el directorio de trabajo
% Cambia los nombres de los archivos según corresponda
dataRobot = readmatrix('DatosRobot/purepursuitEquipo3.csv'); % Datos de la primera simulación
% = readmatrix('DatosGlobales/datos-globales-pp.csv'); % Datos de la segunda simulación
dataGlobal = readmatrix('DatosGlobales/datos-globales-pp2.xlsx');

% Extraer las columnas de coordenadas y ángulo
xRobot = dataRobot(:, 1); % Coordenada X de la simulación 1
yRobot = dataRobot(:, 2); % Coordenada Y de la simulación 1
thetaRobot = dataRobot(:, 3); % Ángulo de la simulación 1

xGlobal = dataGlobal(:, 1); % Coordenada X de la simulación 2
yGlobal= dataGlobal(:, 2); % Coordenada Y de la simulación 2
thetaGlobal = dataGlobal(:, 3); % Ángulo de la simulación 2

% Ajustar el punto de inicio para que ambas simulaciones comiencen en (0, 0)
% Esto depende de cómo están registradas las trayectorias, así que si no es necesario, puedes omitirlo.
xRobot = xRobot - xRobot(1);
yRobot = yRobot - yRobot(1);
xGlobal = xGlobal - xGlobal(1);
yReal = yGlobal - yGlobal(1);

% Crear la figura
figure;

% Graficar trayectoria de referencia
plot(xRef, yRef, 'r-', 'DisplayName', 'Referencia', 'LineWidth', 1);
hold on;

% Graficar la trayectoria de la simulación 1
plot(xRobot, yRobot, 'g-', 'DisplayName', 'Datos Robot', 'LineWidth', 1);
hold on;

% Graficar la trayectoria de la simulación 2
plot(xGlobal, yGlobal, 'c-', 'DisplayName', 'Datos Globales', 'LineWidth', 1);

% Configurar el gráfico
title('Comparativa de Trayectorias Persecucion Pura del Robot');
xlabel('Posición X (m)');
ylabel('Posición Y (m)');
legend('show');
grid on;
axis equal;
