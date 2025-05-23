%Carga de trayectorias
dataRefSquare = readmatrix("Trayectorias/Square.csv");

% Extraer las columnas de coordenadas 
xRef = dataRefSquare(:,2);
yRef = dataRefSquare(:,3);

% Cargar datos de las simulaciones
% Asegúrate de tener los archivos CSV en el directorio de trabajo
% Cambia los nombres de los archivos según corresponda
dataDef = readmatrix('odom_results_parte2_default_square.csv'); % Datos de la primera simulación
dataMod = readmatrix('odom_results_parte2_mod_square.csv'); % Datos de la segunda simulación

% Extraer las columnas de coordenadas y ángulo
xDef = dataDef(:, 1); % Coordenada X de la simulación 1
yDef = dataDef(:, 2); % Coordenada Y de la simulación 1
thetaDef = dataDef(:, 3); % Ángulo de la simulación 1

xMod = dataMod(:, 1); % Coordenada X de la simulación 2
yMod = dataMod(:, 2); % Coordenada Y de la simulación 2
thetaMod = dataMod(:, 3); % Ángulo de la simulación 2

% Ajustar el punto de inicio para que ambas simulaciones comiencen en (0, 0)
% Esto depende de cómo están registradas las trayectorias, así que si no es necesario, puedes omitirlo.
xDef = xDef - xDef(1);
yDef = yDef - yDef(1);
xMod = xMod - xMod(1);
yMod = yMod - yMod(1);

% Crear la figura
figure;

% Graficar trayectoria de referencia
plot(xRef, yRef, 'r-', 'DisplayName', 'Referencia', 'LineWidth', 1);
hold on;

% Graficar la trayectoria de la simulación 1
plot(xDef, yDef, 'g-', 'DisplayName', 'Simulación Default', 'LineWidth', 1);
hold on;

% Graficar la trayectoria de la simulación 2
plot(xMod, yMod, 'c-', 'DisplayName', 'Simulación Mod', 'LineWidth', 1);

% Configurar el gráfico
title('Comparativa de Trayectorias del Robot Persecucion Pura');
xlabel('Posición X (m)');
ylabel('Posición Y (m)');
legend('show');
grid on;
axis equal;

% Añadir etiquetas o detalles adicionales si es necesario