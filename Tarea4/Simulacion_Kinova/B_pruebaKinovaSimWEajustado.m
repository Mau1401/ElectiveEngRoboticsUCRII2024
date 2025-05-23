%Para simulación del robot
clc
clear all
close all
% crear instancia del modelo del robot Gen3 lite
close all
gen3Lite = importrobot("GEN3-LITEef.urdf","MeshPath",{"/gen3_lite/meshes"});
% show(gen3Lite)
%
gen3Lite.DataFormat = 'column';
%q_home = [0 10 150 -90 50 0]'*pi/180;
q_home = [0 20 100 90 90 0]'*pi/180;
eeName = 'END_EFFECTOR';
T_home = getTransform(gen3Lite, q_home, eeName);
T_home(1:4,4) =  [0;0;0;1];

% mostrar robot y configuración home
show(gen3Lite,q_home,"Visuals", "on", 'Frames', 'off'); %
axis auto;
view([60,10]); 
%% definir waypoints
toolPositionHome = [0.15   0.05    0.41]; % [0.27    -0.07    0.36]
waypoints = toolPositionHome' + ... 
            [0 0 0 ; 0.1 0 -0.2 ; 0.15 -0.1 -0.2 ; 0 0 0]'; %[0 0 0 ; -0.05 0.1 0.1 ; 0.1 0 0.05 ; -0.05 -0.1 0.1 ; 0 0 0]'

% orientación
orientations = [(pi/2)             0    (pi); 
                (pi/2)       0    pi;
                (pi/2)              0    pi;
                (pi/2)              0    pi]';
% tiempos
waypointTimes = 0:7:21; %numero de instantes debe coincidir con la dimensión del vector waypoints, en múltiplos de 7 iniciando en 0, ajustar el valor final al agregar más puntos
ts = 0.25;
trajTimes = 0:ts:waypointTimes(end);
% limites velocidad y aceleración
waypointVels = 0.1 *[ 0  1  0;
                      1  0  0;
                      0  1  0;
                      0	 1  0]';

waypointAccels = zeros(size(waypointVels));
waypointAccelTimes = diff(waypointTimes)/4;

% visualizar solución
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(gen3Lite,q_home,'Frames','off','PreservePlot',false);
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);
axis auto;
view([30 15]);
%% ajustar y resolver la cinemática inversa
% resolver cinemática para cada punto
% ajustes del solver
ik = inverseKinematics('RigidBodyTree',gen3Lite);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = q_home';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;
ik.SolverParameters.AllowRandomRestart = false;

includeOrientation = true; 
numWaypoints = size(waypoints,2);
numJoints = numel(gen3Lite.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);
for idx = 1:numWaypoints
    if includeOrientation
        tgtPose = trvec2tform(waypoints(:,idx)') * eul2tform(orientations(:,idx)');
    else
        tgtPose = trvec2tform(waypoints(:,idx)') * eul2tform([pi/2 0 pi/2]); %#ok<UNRCH> 
    end
    [config,info] = ik(eeName,tgtPose,ikWeights',ikInitGuess');
    jointWaypoints(:,idx) = config';
end

% crear trayectoria en el espacio de articulaciones mediante interpolación
trajType = 'trap'; %trap
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(numJoints,numWaypoints));
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(numJoints,numWaypoints), ...
            'AccelerationBoundaryCondition',zeros(numJoints,numWaypoints));
        
    case 'bspline'
        ctrlpoints = jointWaypoints; % Can adapt this as needed
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        % Remove the first velocity sample
        qd(:,1) = zeros (6,1);    
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

%% visualizar la solución
for idx = 1:numel(trajTimes)  
 
    config = q(:,idx)';
    
    % Find Cartesian points for visualization
    eeTform = getTransform(gen3Lite,config',eeName);
    if plotMode == 1
        eePos = tform2trvec(eeTform);
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
    end
 
    % Show the robot
    show(gen3Lite,config','Frames','off','PreservePlot',false);
    axis auto;
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow   
    
end

%% Generación de trayectorias 
%joint position velocity and acceleration 
q = q*180/pi;
qd = qd*180/pi;
qdd = qdd*180/pi;
 
timestamp = (0:0.001:waypointTimes(end))';
qs_deg = interp1(trajTimes',q',timestamp);
vel = interp1(trajTimes',qd',timestamp);
acc = interp1(trajTimes',qdd',timestamp);

% Graficar las referencias de posición, velocidad y aceleración
figure;
subplot(3, 1, 1);
plot(timestamp, qs_deg);
xlabel('Tiempo [s]');
ylabel('Posición Articular [°]');
title(['Referencia de Posición Articular - ' trajType]);
grid on;

subplot(3, 1, 2);
plot(timestamp, vel);
xlabel('Tiempo [s]');
ylabel('Velocidad Articular [°/s]');
title(['Referencia de Velocidad Articular - ' trajType]);
grid on;

subplot(3, 1, 3);
plot(timestamp, acc);
xlabel('Tiempo [s]');
ylabel('Aceleración Articular [°/s²]');
title(['Referencia de Aceleración Articular - ' trajType]);
grid on;
