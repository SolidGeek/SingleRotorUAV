function data = extract_tlm_package( package )
    % Unpack measurements
    data.gx = typecast(uint8(package(2:5)), 'single');  % float32
    data.gy = typecast(uint8(package(6:9)), 'single');  % float32
    data.gz = typecast(uint8(package(10:13)), 'single'); % float32

    data.roll     = typecast(uint8(package(14:17)), 'single'); % float32
    data.pitch    = typecast(uint8(package(18:21)), 'single'); % float32
    data.yaw      = typecast(uint8(package(22:25)), 'single'); % float32

    data.ax = typecast(uint8(package(26:29)), 'single'); % float32
    data.ay = typecast(uint8(package(30:33)), 'single'); % float32
    data.az = typecast(uint8(package(34:37)), 'single'); % float32

    data.vx = typecast(uint8(package(38:41)), 'single'); % float32
    data.vy = typecast(uint8(package(42:45)), 'single'); % float32
    data.z = typecast(uint8(package(46:49)), 'single'); % float32

    status = package(50);
    data.status_imu = bitget( status, 1);
    data.status_flow = bitget( status, 2);
    data.status_lidar = bitget( status, 3);
    
    % Unpack estimates
    data.xe = typecast(uint8(package(51:54)), 'single'); % float32
    data.ye = typecast(uint8(package(55:58)), 'single'); % float32
    data.ze = typecast(uint8(package(59:62)), 'single'); % float32
    
    data.vxe = typecast(uint8(package(63:66)), 'single'); % float32
    data.vye = typecast(uint8(package(67:70)), 'single'); % float32
    data.vze = typecast(uint8(package(71:74)), 'single'); % float32
    
    % Unpack actuation signals
    data.a1 = typecast(uint8(package(75:78)), 'single'); % float32
    data.a2 = typecast(uint8(package(79:82)), 'single'); % float32
    data.a3 = typecast(uint8(package(83:86)), 'single'); % float32
    data.a4 = typecast(uint8(package(87:90)), 'single'); % float32
    data.dshot = typecast(uint8(package(91:92)), 'uint16'); % uint16
end