% First run record.m, to sample data from UDP
% Next run this 


%% Data Import
z = [data.z];
az = [data.az];

%% Kalman filter design parameters

% Sample time
dt = 0.01; 

% System Matrix
A = [ 1  dt ;  % dz
      0  1 ];  % dvz

% Input Matrix
B = [ 0.5*dt^2  ;  % z
      dt       ]; % vz

% Measurements Matrix
C = [ 1 0 ];  

% Calculate discrete steady-state kalman gain

% Process Noice Covariance
sigma_acc = 0.001; % Variance of acceleration m/s^2
Q = [ dt^4/4     dt^3/2  ;
      dt^3/2     dt^2   ] * sigma_acc;

% Measurement Noise Covariance
R = [10^-8]; 

G = eye(2)*1;
kf = dlqe(A,G,C,Q,R)

%% Run Kalman Filter

n = 2000;
x = zeros(n, 2);

x(1,:) = [ z(1); 0];

for i = 2:n
    
    u = az(i);
    y = 0;
    
    H = zeros(1,2);

    if( data(i).status_lidar == 1 )
        H(1,1) = 1;
        y(1) = z(i);
    end
    
    xpre = A*x(i-1,:)' + B*u;             % Predicted state estimate
    x(i,:) = xpre + kf*(y - H*xpre);   % Updated state estimate
end

%% Plotting

figure(1)
subplot(2,1,1)
hold on
plot( x(:,1), '.','LineWidth', 2 );
plot( z , 'LineWidth', 2);
hold off
title("Altitude (z-axis)");
legend("Kalman", "Measured");

subplot(2,1,2)
hold on
plot( x(:,2) );
hold off
title("Altitude velocity (z-axis)");
legend("Kalman");
