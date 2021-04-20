close all
clear
% Start UDP server on port 8888

u = udpport("datagram", "LocalPort", 8888);


% Kalman filter settings

dt = 0.01; % 100 Hz

% --- Covariance matrixes ---
% Process noise 
sigma = 2;
Q = [dt^4/4  0       0       dt^3/2 ;   % p_z
     0       dt^2    0       0      ;   % v_x
     0       0       dt^2    0      ;   % v_y
     dt^3/2  0       0       dt^2   ] * sigma;  % v_z

% Measurement noise
R = [1/100  0      0      0;   % LIDAR p_z
     0      1/400  0      0;   % VO v_x       
     0      0      1/400  0;   % VO v_y
     0      0      0      1]; % 

% --- System matrixes ---
A = [ 1  0  0  dt ;  % dz
      0  1  0  0  ;  % dvx
      0  0  1  0  ;  % dvy
      0  0  0  1 ];  % dvz

%     ax        ay        az      
B = [ 0         0         0.5*dt^2  ;  % z
      dt        0         0         ;  % vx
      0         dt        0         ;  % vy
      0         0         dt        ]; % vz
  
C = [ 1 0 0 0; 
      0 1 0 0;
      0 0 1 0;
      0 0 0 0];  
  
% Calculate steady state kalman gain
Ts = 14;
sys = ss(A,B,C,zeros(4,3));
G = eye(4)*1;
global kf;
kf = dlqe(A,G,C,Q,R);
% lqe(A,Q,C,Q,R)

%% 

x = [0.08 0 0 0]';   % Initial states
x2 = x;
P = Q;                   % Initial error coveriance

% Plot Setup

figure(1)

% accel_plot = subplot(2,2,1);
% ylabel(accel_plot, 'm/s^2');
% ax_line = line(accel_plot,1:100,zeros(100,1), 'Color', 'Red');
% ay_line = line(accel_plot,1:100,zeros(100,1), 'Color', 'Blue');
% az_line = line(accel_plot,1:100,zeros(100,1), 'Color', 'Green');
% stripchart('Initialize', accel_plot, 'samples');

vel_plot = subplot(1,1,1);
ylabel(vel_plot, 'm/s');
ylim(vel_plot, [-0.5 0.5]);
vx_line = line(vel_plot,1:200,zeros(200,1), 'Color', 'Red');
vy_line = line(vel_plot,1:200,zeros(200,1), 'Color', 'Blue');
vz_line = line(vel_plot,1:200,zeros(200,1), 'Color', 'Green');
stripchart('Initialize', vel_plot, 'samples');


while( 1 )
    
    % Each time a new packet is available
    if( u.NumDatagramsAvailable )

        % Read one package from 
        package = read(u,1);
        data = extract_tlm( package.Data );
        
        % Build H matrix, to indicate which measurements are ready
        H = zeros(4,4);
        
        if( data.status_flow == 1 )
            H(2:3, 2:3) = eye(2);
        end
        if( data.status_lidar == 1 )
            H(1,1) = 1;
        end

        % Input and measurement vector
        input = [-data.ax; -data.ay; data.az];
        % input = [0; 0; 0];
        
        measurements = [data.z; data.vx; data.vy; 0];
        
        [x, P] = discrete_kalman_filter( x, P, input, measurements, H, A, B, Q, R );
        x2 = discrete_ss_kalman_filter( x2, input, measurements, H, A, B );
%         stripchart('Update',ax_line, input(1));
%         stripchart('Update',ay_line, input(2));
%         stripchart('Update',az_line, input(3));
 
        stripchart('Update',vx_line, x(2) );
        stripchart('Update',vy_line, x2(2) );
        % stripchart('Update',vz_line, x(4));
        drawnow % refresh display

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

% u = input, z = measurement vector, h = measurement mapping 
% x = [x y z vx vy vz]
% u = [ax ay az]
function [x_new, P_new] = discrete_kalman_filter( x_last, P_last, u, z, H, A, B, Q, R ) 
      
    % Prediction step
    xpre = A*x_last + B*u;     % Predicted state estimate
    Ppre = A*P_last*A' + Q;    % Predicted error covariance

    % Updating step
    k     = Ppre*H'*inv(R + H*Ppre*H'); % Kalman gain
    x_new = xpre + k*(z - H*xpre);     % Updated state estimate
    P_new = (eye(4) - k*H)*Ppre;       % Updated error covariance
end

function [x_new] = discrete_ss_kalman_filter( x_last, u, z, H, A, B)

    global kf

    xpre = A*x_last + B*u;     % Predicted state estimate
    x_new = xpre + kf*(z - H*xpre);     % Updated state estimate
    
end