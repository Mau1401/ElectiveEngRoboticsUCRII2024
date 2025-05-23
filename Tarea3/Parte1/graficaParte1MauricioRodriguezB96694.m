% Cargar datos de las simulaciones
% Asegúrate de tener los archivos CSV en el directorio de trabajo
% Cambia los nombres de los archivos según corresponda
data1 = readmatrix('odom_results_default.csv'); % Datos de la primera simulación
data2 = readmatrix('odom_results_mod.csv'); % Datos de la segunda simulación

% Extraer las columnas de coordenadas y ángulo
x1 = data1(:, 1); % Coordenada X de la simulación 1
y1 = data1(:, 2); % Coordenada Y de la simulación 1
theta1 = data1(:, 3); % Ángulo de la simulación 1

x2 = data2(:, 1); % Coordenada X de la simulación 2
y2 = data2(:, 2); % Coordenada Y de la simulación 2
theta2 = data2(:, 3); % Ángulo de la simulación 2

% Ajustar el punto de inicio para que ambas simulaciones comiencen en (0, 0)
% Esto depende de cómo están registradas las trayectorias, así que si no es necesario, puedes omitirlo.
x1 = x1 - x1(1);
y1 = y1 - y1(1);
x2 = x2 - x2(1);
y2 = y2 - y2(1);

% Crear la figura
figure;

% Graficar la trayectoria de la simulación 1
plot(x1, y1, 'b-', 'DisplayName', 'Simulación 1', 'LineWidth', 1);
hold on;

% Graficar la trayectoria de la simulación 2
plot(x2, y2, 'y-', 'DisplayName', 'Simulación 2', 'LineWidth', 1);

% Definir el obstáculo como un cuadrado rojo
% Posición y tamaño del cuadrado
obstaculo_x = 2.5;  % Coordenada X del centro del obstáculo
obstaculo_y = 0;  % Coordenada Y del centro del obstáculo
size = 1;         % Tamaño del lado del cuadrado (puedes ajustarlo)

% Coordenadas de los 4 vértices del cuadrado
x_obst = obstaculo_x + [-0.5 0.5 0.5 -0.5] * size;  % Vértices X
y_obst = obstaculo_y + [-0.5 -0.5 0.5 0.5] * size;  % Vértices Y

% Cerrar el cuadrado conectando el último vértice con el primero
x_obst = [x_obst, x_obst(1)];
y_obst = [y_obst, y_obst(1)];

% Graficar el cuadrado rojo como obstáculo
plot(x_obst, y_obst, 'r-', 'LineWidth', 2, 'DisplayName', 'Obstáculo');
% Configurar el gráfico
title('Comparativa de Trayectorias del Robot');
xlabel('Posición X (m)');
ylabel('Posición Y (m)');
legend('show');
grid on;
axis equal;

% Añadir etiquetas o detalles adicionales si es necesario
