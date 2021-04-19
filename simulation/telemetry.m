close all
clear
% Start UDP server on port 8888

u = udpport("datagram", "LocalPort", 8888);


figure(1)

accel_plot = subplot(2,2,1);
ylabel(accel_plot, 'm/s^2');
ax_line = line(accel_plot,1:100,zeros(100,1), 'Color', 'Red');
ay_line = line(accel_plot,1:100,zeros(100,1), 'Color', 'Blue');
az_line = line(accel_plot,1:100,zeros(100,1), 'Color', 'Green');
stripchart('Initialize', accel_plot, 'samples');


vel_plot = subplot(2,2,2);
ylabel(vel_plot, 'm/s');
vx_line = line(vel_plot,1:200,zeros(200,1), 'Color', 'Red');
vy_line = line(vel_plot,1:200,zeros(200,1), 'Color', 'Blue');
vz_line = line(vel_plot,1:200,zeros(200,1), 'Color', 'Green');
stripchart('Initialize', vel_plot, 'samples');


while( 1 )

    if( u.NumDatagramsAvailable )
       u.NumDatagramsAvailable
       % Read one package from 
       package = read(u,1);
       data = extract_tlm( package.Data );
       

       stripchart('Update',ax_line, data.ax);
       stripchart('Update',ay_line, data.ay);
       stripchart('Update',az_line, data.az);
       
       stripchart('Update',vx_line, data.vx);
       stripchart('Update',vy_line, data.vy);
       stripchart('Update',vz_line, data.vz);
       drawnow                                 % refresh display
    end
    
end

function data = extract_tlm( bytes )

    data.gx = typecast(uint8(bytes(2:5)), 'single');  % float32
    data.gy = typecast(uint8(bytes(6:9)), 'single');  % float32
    data.gz = typecast(uint8(bytes(10:13)), 'single'); % float32

    data.roll     = typecast(uint8(bytes(14:17)), 'single'); % float32
    data.pitch    = typecast(uint8(bytes(18:21)), 'single'); % float32
    data.yaw      = typecast(uint8(bytes(22:25)), 'single'); % float32

    % qw, qi, qj, qk = 26:41 - 16 bytes not used 

    data.ax = typecast(uint8(bytes(42:45)), 'single'); % float32
    data.ay = typecast(uint8(bytes(46:49)), 'single'); % float32
    data.az = typecast(uint8(bytes(50:53)), 'single'); % float32

    data.vx = typecast(uint8(bytes(54:57)), 'single'); % float32
    data.vy = typecast(uint8(bytes(58:61)), 'single'); % float32
    data.vz = typecast(uint8(bytes(62:65)), 'single'); % float32

    data.x = typecast(uint8(bytes(66:69)), 'single'); % float32
    data.y = typecast(uint8(bytes(70:73)), 'single'); % float32
    data.z = typecast(uint8(bytes(74:77)), 'single'); % float32

    data.a1 = typecast(uint8(bytes(78:81)), 'single'); % float32
    data.a2 = typecast(uint8(bytes(82:85)), 'single'); % float32
    data.a3 = typecast(uint8(bytes(86:89)), 'single'); % float32
    data.a4 = typecast(uint8(bytes(90:93)), 'single'); % float32
    data.dshot = typecast(uint8(bytes(94:95)), 'uint16'); % uint16
    
end