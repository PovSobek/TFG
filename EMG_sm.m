%Iniciar ROS
setenv('ROS_DOMAIN_ID', '0')
setenv('ROS_IP', '172.30.134.19')

node = ros2node("/matlab_node")
transicion = ros2publisher(node, "/transition", "std_msgs/String")

msg = ros2message(transicion);

%Iniciar Noraxon
[stream_config, sensor_selection] = noraxon_stream_init('127.0.0.1', '9220');
tiempo = 0;
msg.data = 'b';
threshold = 50;
send(transicion, msg)

tic

while tiempo <= 180 %segundos
    data = noraxon_stream_collect(stream_config, 0.3);
    f_amp=mean(data(1).samples(:));
    e_amp=mean(data(2).samples(:));
    
    if f_amp < threshold && e_amp < threshold
        msg.data = 'b'
        send(transicion, msg)
    elseif f_amp >= threshold && e_amp < threshold
        msg.data = 'a'
        send(transicion, msg)
    elseif f_amp < threshold && e_amp >= threshold
        msg.data = 'c'
        send(transicion, msg)
    end

    toc
end
