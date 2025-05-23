% Parámetros iniciales
q_s0 = 0.06;  % Posición inicial (rads)
v_s0 = 0.06;  % Velocidad inicial (rads/s)
q_e1 = 0.58;  % Posición final (rads)
v_e1 = 0.00;  % Velocidad final (rads/s)
a = 0.56;     % Aceleración (rads/s^2)
v_max = 0.3;  % Velocidad máxima permitida (rads/s)

% a) Calcular la velocidad alcanzable (v_com)
v_com = sqrt(v_s0^2 + 2 * a * (q_e1 - q_s0));

% Verificar si se puede alcanzar la velocidad máxima
if v_com > v_max
    fprintf('La velocidad máxima (%0.2f rads/s) es alcanzable.\n', v_max);
    v_com = v_max; % Limitar a la velocidad máxima
else
    fprintf('La velocidad alcanzable es menor a v_max: %0.2f rads/s.\n', v_com);
end

% b) Primera posición donde se alcanza v_max (q_e0)
q_e0 = q_s0 + (v_max^2 - v_s0^2) / (2 * a);

% c) Posición donde comienza la desaceleración (q_s1)
q_s1 = q_e1 - v_max^2 / (2 * a);

% d) Tiempo para alcanzar la velocidad máxima (t_acc)
t_acc = (v_max - v_s0) / a;

% Resultados
fprintf('Velocidad alcanzable (v_com): %0.2f rads/s\n', v_com);
fprintf('Primera posición donde se alcanza v_max (q_e0): %0.2f rads\n', q_e0);
fprintf('Posición donde comienza la desaceleración (q_s1): %0.2f rads\n', q_s1);
fprintf('Tiempo para alcanzar v_max (t_acc): %0.2f s\n', t_acc);
