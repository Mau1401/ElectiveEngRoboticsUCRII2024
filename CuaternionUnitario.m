function quaternion = rotationMatrixToQuaternion(R)
    % Verifica si la matriz de rotación es válida
    %if abs(det(R) - 1.0) > 1e-6 || norm(R * R' - eye(3)) > 1e-6
    %    error('La matriz de rotación no es válida. Debe ser ortogonal y tener determinante 1.');
    %end

    % Calcular el trazo (trace) de la matriz de rotación
    trace_R = R(1,1) + R(2,2) + R(3,3);

    if trace_R > 0
        S = sqrt(trace_R + 1.0) * 2; % S = 4 * q0
        q0 = 0.25 * S;
        q1 = (R(3,2) - R(2,3)) / S;
        q2 = (R(1,3) - R(3,1)) / S;
        q3 = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S = 4 * q1
        q0 = (R(3,2) - R(2,3)) / S;
        q1 = 0.25 * S;
        q2 = (R(1,2) + R(2,1)) / S;
        q3 = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S = 4 * q2
        q0 = (R(1,3) - R(3,1)) / S;
        q1 = (R(1,2) + R(2,1)) / S;
        q2 = 0.25 * S;
        q3 = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S = 4 * q3
        q0 = (R(2,1) - R(1,2)) / S;
        q1 = (R(1,3) + R(3,1)) / S;
        q2 = (R(2,3) + R(3,2)) / S;
        q3 = 0.25 * S;
    end

    % Formar el cuaternión resultante
    quaternion = [q0, q1, q2, q3];
end

R = [0.75 -0.0473672 -0.65974;
     0.433013 0.789149 0.435596;
     0.5 -0.612372 0.612372];

quaternion = rotationMatrixToQuaternion(R);
disp('El cuaternión unitario es: ');
disp(quaternion);