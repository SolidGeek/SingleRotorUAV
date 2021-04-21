clear 
u = udpport("datagram", "LocalPort", 8888);

index = 0;
n = 2000;

while( index < n )
    % Each time a new packet is available
    if( u.NumDatagramsAvailable )
       
        index = index + 1;
        index

        % Read one package from 
        package = read(u,1);
        data(index) = extract_tlm( package.Data );
        
    end

end

function data = extract_tlm( package )
    % Unpack measurements
    data.gx = typecast(uint8(package(2:5)), 'single');  % float32
    data.gy = typecast(uint8(package(6:9)), 'single');  % float32
    data.gz = typecast(uint8(package(10:13)), 'single'); % float32

    data.roll     = typecast(uint8(package(14:17)), 'single'); % float32
    data.pitch    = typecast(uint8(package(18:21)), 'single'); % float32
    data.yaw      = typecast(uint8(package(22:25)), 'single'); % float32

    % qw, qi, qj, qk = 26:41 - 16 bytes not used 

    data.ax = typecast(uint8(package(42:45)), 'single'); % float32
    data.ay = typecast(uint8(package(46:49)), 'single'); % float32
    data.az = typecast(uint8(package(50:53)), 'single'); % float32

    data.vx = typecast(uint8(package(54:57)), 'single'); % float32
    data.vy = typecast(uint8(package(58:61)), 'single'); % float32
    data.vz = typecast(uint8(package(62:65)), 'single'); % float32

    data.x = typecast(uint8(package(66:69)), 'single'); % float32
    data.y = typecast(uint8(package(70:73)), 'single'); % float32
    data.z = typecast(uint8(package(74:77)), 'single'); % float32

    status = package(78);
    data.status_imu = bitget( status, 1);
    data.status_flow = bitget( status, 2);
    data.status_lidar = bitget( status, 3);
    
    % Unpack actuation signals
    data.a1 = typecast(uint8(package(79:82)), 'single'); % float32
    data.a2 = typecast(uint8(package(83:86)), 'single'); % float32
    data.a3 = typecast(uint8(package(87:90)), 'single'); % float32
    data.a4 = typecast(uint8(package(91:94)), 'single'); % float32
    data.dshot = typecast(uint8(package(95:96)), 'uint16'); % uint16
end