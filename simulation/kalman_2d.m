% First run record.m, to sample data from UDP
% Next run this 

% load('walk_test_data_1.mat')


%% Data Import
z = [data.z];
% 
vx = [data.vx]*0.5;
vy = [data.vy]*0.5;

ax = [data.ax];
ay = [data.ay];
az = [data.az];

%% Kalman filter design parameters

% State Vectir
% x = [x y z vx vy vz]

% Sample time
dt = 0.01; 

% System Matrix
A = [ 1  0  0  dt 0  0  ;  % dx
      0  1  0  0  dt 0  ;  % dy 
      0  0  1  0  0  dt ;  % dz
      0  0  0  1  0  0  ;  % dvx
      0  0  0  0  1  0  ;  % dvy
      0  0  0  0  0  1 ];  % dvz

% Input Matrix
B = [ 0.5*dt^2  0         0         ;  % dx
      0         0.5*dt^2  0         ;  % dy
      0         0         0.5*dt^2  ;  % dz
      dt        0         0         ;  % dvx
      0         dt        0         ;  % dvy
      0         0         dt        ]; % dvz

% Measurements Matrix
C = [ 1 0 0 0 0 0  ; % x 
      0 1 0 0 0 0  ; % y
      0 0 1 0 0 0  ; % z
      0 0 0 1 0 0  ; % vx
      0 0 0 0 1 0  ; % vy
      0 0 0 0 0 1 ]; % vz

% Calculate discrete steady-state kalman gain

% Process Noice Covariance
sigma_acc = 0.01; % Variance of acceleration m/s^2
% Q = [dt^4/4   0      0        dt^3/2 ;
%      0        dt^2   0        0      ;
%      0        0      dt^2     0      ;
%      dt^3/2   0      0        dt^2   ] * sigma_acc;

Q = [dt^4/4  0       0       dt^3/2  0       0      ;
     0       dt^4/4  0       0       dt^3/2  0      ;
     0       0       dt^4/4  0       0       dt^3/2 ;
     dt^3/2  0       0       dt^2    0       0      ;
     0       dt^3/2  0       0       dt^2    0      ;
     0       0       dt^3/2  0       0       dt^2   ] * sigma_acc;

% Measurement Noise Covariance
R = [1      0      0        0        0        0 ;
     0      1      0        0        0        0 ;
     0      0      5*10^-7  0        0        0 ; % Lidar
     0      0      0        8*10^-5  0        0 ; % Flow x
     0      0      0        0        8*10^-5  0 ; % Flow y
     0      0      0        0        0        1 ];

G = eye(6)*1;
kf = dlqe(A,G,C,Q,R)

%% Run Kalman Filter

n = length(data);
x = zeros(n, 6); % State vector [ z vx vy vz ]

x(1,:) = [ 0; 0; z(1); 0; 0; 0 ];

for i = 2:n
    
    u = [ ax(i); ay(i); az(i)];
    y = [ 0; 0; 0 ; 0; 0; 0 ];
    
    H = zeros(6,6);

    if( data(i).status_lidar == 1 )
        H(3,3) = 1;
        y(3) = z(i);
    end
    
    if( data(i).status_flow == 1 )
        H(4:5, 4:5) = eye(2);
        y(4) = vx(i);
        y(5) = vy(i);
    end
    
    xpre = A*x(i-1,:)' + B*u;             % Predicted state estimate
    x(i,:) = xpre + kf*(y - H*xpre);   % Updated state estimate
end

%% Plotting

figure(1)
subplot(2,2,1)
hold on
plot( z , 'LineWidth', 2);
plot( x(:,3), '.' );
plot( [data.ze] );
hold off
grid on
title("Altitude (z-axis)");
legend("Measured", "Kalman");

subplot(2,2,2)
hold on
plot( x(:,6) );
plot( [data.vze] );
hold off
grid on
title("Altitude velocity (z-axis)");
legend("Kalman");

subplot(2,2,3)
hold on
plot( vx, 'LineWidth', 2  );
plot( x(:,4), '.' );
hold off
grid on
title("Velocity (x-axis)");
legend("Flow", "Kalman");

subplot(2,2,4)
hold on
plot( vy, 'LineWidth', 2  );
plot( x(:,5), '.');
hold off
grid on
title("Velocity (y-axis)");
legend("Flow", "Kalman");

figure(2)
plot3(x(:,1), x(:,2), x(:,3));
title("Position (world)");
legend("Kalman");
xlabel("x [m]");
ylabel("y [m]");
grid on