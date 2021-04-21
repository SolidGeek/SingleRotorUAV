% First run record.m, to sample data from UDP
% Next run this 


%% Data Import
ax = [data.ax];
ay = [data.ay];
az = [data.az];

vx = [data.vx];
vy = [data.vy];

z = [data.z];


%% Kalman filter design parameters

% Sample time
dt = 0.01; 

% Process Noice Covariance
sigma_acc = 0.0010; % Variance of accelerometer
Q = [dt^4/4   0      0        dt^3/2 ;
     0        dt^2   0        0      ;
     0        0      dt^2     0      ;
     dt^3/2   0      0        dt^2   ] * sigma_acc;

% Measurement Noise Covariance
R = [10^-8  0      0      ;   % LIDAR
     0      10^-4  0      ;   % VOx      
     0      0      10^-4 ];   % VOy

% System Matrix
A = [ 1  0  0  dt ;  % dz
      0  1  0  0  ;  % dvx
      0  0  1  0  ;  % dvy
      0  0  0  1 ];  % dvz

% Input Matrix
B = [ 0         0         0.5*dt^2  ;  % z
      dt        0         0         ;  % vx
      0         dt        0         ;  % vy
      0         0         dt        ]; % vz

% Measurements Matrix
C = [ 1 0 0 0  ; 
      0 1 0 0  ;
      0 0 1 0 ];  
  
% Calculate discrete steady-state kalman gain
G = eye(4)*1;
kf = dlqe(A,G,C,Q,R);

%% Run Kalman Filter

n = 2000;
x = zeros(n, 4);

dz = zeros(n,1);

x(1,:) = [ z(1); 0; 0; 0];



for i = 2:n
    
    dz(i) = dz(i-1) + (z(i) - z(i-1))/dt;
    
    u = [-ax(i); -ay(i); az(i)];
    y = [z(i); vx(i); vy(i); 0];
    
    H = zeros(4,4);
    
    if( data(i).status_flow == 1 )
        H(2:3, 2:3) = eye(2);
    end
    if( data(i).status_lidar == 1 )
        H(1,1) = 1;
    end
    
    xpre = A*x(i-1,:)' + B*u;     % Predicted state estimate
    x(i,:) = xpre + kf*(y - H*xpre);     % Updated state estimate
    
end

%% Plotting

figure(1)
subplot(2,2,1)
hold on
plot( x(:,1) );
plot( z );
hold off
title("Altitude (z-axis)");
legend("Kalman", "Measured");

subplot(2,2,2)
hold on
plot( x(:,4) );
plot( dz );
hold off
title("Altitude velocity (z-axis)", "Raw");
legend("Kalman");

subplot(2,2,3);
hold on
plot( x(:,2) );
plot( vx );
hold off
title("Velocity (x-axis)");
legend("Kalman", "Measured");

subplot(2,2,4);
hold on
plot( x(:,3) );
plot( vy );
hold off
title("Velocity (y-axis)");
legend("Kalman", "Measured");