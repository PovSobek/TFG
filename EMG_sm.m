% Iniciar ROS
setenv('ROS_DOMAIN_ID', '0')
setenv('ROS_IP', '172.30.134.19')
node = ros2node("/matlab_node");

% Publishers
transicion = ros2publisher(node, "/transition", "std_msgs/String");
proporcion = ros2publisher(node, "/proportional", "std_msgs/Float64");
msg_t = ros2message(transicion);
msg_p = ros2message(proporcion);

% Iniciar Noraxon
[stream_config, sensor_selection] = noraxon_stream_init('127.0.0.1', '9220');

% Variables de control
threshold = 10;
mvc = 100;
msg_t.data = 'b';
msg_p.data = 0.0;
prev_st = 'b'; % Estado inicial

% Enviar estado inicial
send(transicion, msg_t)
send(proporcion, msg_p)

tic;
tiempo = 0;

while tiempo <= 180 % segundos
    tiempo = toc; % Actualizar el tiempo real del bucle

    data = noraxon_stream_collect(stream_config, 0.1);
    if isempty(data), continue; end % Evitar errores si no hay datos
    
    f_amp = mean(data(1).samples(:));
    e_amp = mean(data(2).samples(:));
    
    % Calcular proporción señal sobre MVC
    msg_p.data = min(f_amp / mvc, 1.0);
    
    % LÓGICA DE ESTADOS
    % 1. Determinar el estado
    if f_amp < threshold && e_amp < threshold
        current_target = 'b';
    elseif f_amp >= threshold && e_amp < threshold
        current_target = 'd';
    elseif f_amp < threshold && e_amp >= threshold
        current_target = 'c';
    elseif f_amp >= threshold && e_amp >= threshold
        current_target = 'a';
    end
    
    % 2. Aplicar restricción: Solo cambiar a a, c, d si venimos de 'b'
    if current_target == 'b'
        msg_t.data = 'b';
    else
        if prev_st == 'b'
            msg_t.data = current_target;
        else
            msg_t.data = prev_st; 
        end
    end
    
    % 3. Publicar datos
    send(transicion, msg_t);
    send(proporcion, msg_p);
    
    % Actualizar estado previo para la siguiente iteración
    prev_st = msg_t.data;
    
    fprintf('Tiempo: %.2f | Estado: %s | F: %.2f | E: %.2f\n', tiempo, msg_t.data, f_amp, e_amp);
end
