%Para simulación del robot
clc
clear all
close all
% crear instancia del modelo del robot GEN3 lite
gen3Lite = importrobot("GEN3-LITEef.urdf","MeshPath",{"/gen3_lite/meshes"});
show(gen3Lite)

%
gen3Lite.DataFormat = 'column';
q_home = [0 -20 120 -90 50 00]'*pi/180;
eeName = 'END_EFFECTOR';
T_home = getTransform(gen3Lite, q_home, eeName);
T_home(1:4,4) =  [0;0;0;1];

% mostrar robot y configuración home
show(gen3Lite,q_home,"Visuals", "on", 'Frames', 'off');
axis auto;
view([60,10]); 
%% definir waypoints

center = [0.45 -0.06 0.31]; %[x y z] % ubicación del elemento final [0.45 -0.06 0.31]
radius = 0.1;

dt = 0.25;
t = (0:dt:10)';
theta = t*(2*pi/t(end))-(pi/2);
points = center + radius*[0*ones(size(theta)) cos(theta) sin(theta)];

hold on;
plot3(points(:,1),points(:,2),points(:,3),'-*g', 'LineWidth', 1.5);
xlabel('x');
ylabel('y');
zlabel('z');
axis auto;
view([60,10]);
grid('minor');

%% ajustar parámetros de la cinemática inversa
ik = inverseKinematics('RigidBodyTree',gen3Lite);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1, 1, 1, 1, 1, 1];
q_init = q_home;

% resolver cinemática para cada punto
numJoints = size(q_home,1);   
numWaypoints = size(points,1);
qs = zeros(numWaypoints,numJoints);
for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = points(i,:)';
    [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
    
    % Display status of ik result
    %disp(q_info.Status);
    
    % Store the configuration
    qs(i,:) = q_sol(1:numJoints); 
    
    % Start from prior solution
    q_init = q_sol;
end

%% visualizar la solución
figure; set(gcf,'Visible','on');
ax = show(gen3Lite,qs(1,:)');
ax.CameraPositionMode='auto';
hold on;
 
% Plot waypoints
plot3(points(:,1),points(:,2),points(:,3),'-g','LineWidth',2);
axis auto;
view([60,10]);
grid('minor');
hold on;
 
title('Simulated Movement of the Robot');
% Animate
framesPerSecond = 30;
r = robotics.Rate(framesPerSecond);
for i = 1:numWaypoints
    show(gen3Lite, qs(i,:)','PreservePlot',false);
    drawnow;
    waitfor(r);
end

%% Generación de trayectorias 
%Calculate joint velocity and acceleration at each waypoint using the numerical differentiation
qs_deg = qs*180/pi;
vel = diff(qs_deg)/dt;
vel(1,:) = 0;
vel(end+1,:) = 0;
acc = diff(vel)/dt;
acc(1,:) = 0;
acc(end+1,:) = 0;

%Interpolate the joint position, velocity and acceleration to ensure the 0.001 seconds time step between two trajectory points
timestamp = 0:0.001:t(end);
qs_deg = interp1(t,qs_deg,timestamp);
vel = interp1(t,vel,timestamp);
acc = interp1(t,acc,timestamp);

%% Gráficas de las referencias de posición, velocidad y aceleración
% Posición articular
figure;
subplot(3, 1, 1);
plot(timestamp, qs_deg);
xlabel('Tiempo [s]');
ylabel('Posición Articular [°]');
title('Referencia de Posición Articular');
grid on;

% Velocidad articular
subplot(3, 1, 2);
plot(timestamp, vel);
xlabel('Tiempo [s]');
ylabel('Velocidad Articular [°/s]');
title('Referencia de Velocidad Articular');
grid on;

% Aceleración articular
subplot(3, 1, 3);
plot(timestamp, acc);
xlabel('Tiempo [s]');
ylabel('Aceleración Articular [°/s²]');
title('Referencia de Aceleración Articular');
grid on;
