clc
clear
close all

%% Data Import
load('walk_test_with_kalman_02_05_2021')

% Ground truth
x_vicon = [data.xe];
y_vicon = [data.ye];
z_vicon = [data.ze];
x_vicon = x_vicon - x_vicon(1);
y_vicon = y_vicon - y_vicon(1);
z_vicon = z_vicon - z_vicon(1);

% Onboard estimator
x_est = [data.xt];
y_est = [data.yt];
z_est = [data.zt];

% Measured IMU attitude
roll = [data.roll];
pitch = [data.pitch];
yaw = [data.yaw];

time = double([data.time]);
time = (time - time(1))/100000;

% Measured height
z = [data.z];

% Measured flow data
vx = [data.vx];
vy = [data.vy];

% Estimated velocity
vx_est = [data.vxt];
vy_est = [data.vyt];

% Measured acceleration
ax = [data.ax];
ay = [data.ay];
az = [data.az];

% Measured angular velocity
gx = [data.gx];
gy = [data.gy];


%% Kalman filter design parameters

% State Vectir
% x = [x y z vx vy vz]

% Sample time
dt = 0.005; 

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


% Process Noice Covariance
Q = [ 5e-5  0     0      0     0     0     ;  % x
      0     5e-5  0      0     0     0     ;  % y 
      0     0     1e-5   0     0     0     ;  % z 
      0     0     0      3e-4  0     0     ;  % vx
      0     0     0      0     3e-4  0     ;  % vy
      0     0     0      0     0     5e-3 ];  % vz

% Measurement Noise Covariance
R = [1e-4    0     0      0      0      0 ;
     0      1e-4   0      0      0      0 ;
     0      0      1e-4   0      0      0 ; % Lidar / TOF
     0      0      0      1e-2   0      0 ; % Flow x
     0      0      0      0      1e-2   0 ; % Flow y
     0      0      0      0      0      1 ];

% Calculate discrete steady-state kalman gain
G = eye(6)*1;
kf = dlqe(A,G,C,Q,R);
kf = round( kf, 5 );

matrix_to_cpp(kf) % For easy loading in flight software: C++

%% Run Steady State Kalman Filter

n = length(data);
x = zeros(n, 6); % State vector [ x y z vx vy vz ]

x(1,:) = [ 0; 0; z(1); 0; 0; 0 ];

for i = 2:n
    
    p = [0; 0; z(i)];
    p = rotate_to_world( p, roll(i), pitch(i), yaw(i) );
    h = p(3); % Height

    a = [ ax(i); ay(i); az(i)];
    u = rotate_to_world( a, roll(i), pitch(i), yaw(i) );
    
    y = [ 0; 0; 0 ; 0; 0; 0 ];
    H = zeros(6,6);

    % Vicon at 5 Hz-ish
    if( data(i).stat_pos == 1 )
        H(1:2, 1:2) = eye(2);
        y(1) = x_vicon(i);
        y(2) = y_vicon(i);
    end

    if( data(i).stat_lidar == 1 )
        H(3,3) = 1;
        y(3) = h;
    end
    
    if( data(i).stat_flow == 1 )
        
        v = [( vx(i) - gy(i) ) * h;
             ( vy(i) - gx(i) ) * h;
             0 ];
        v = rotate_to_world( v, roll(i), pitch(i), yaw(i) );

        H(4:5, 4:5) = eye(2);
        y(4) = v(1);
        y(5) = v(2);
    end
 
    xpre = A*x(i-1,:)' + B*u;          % Predicted state estimate
    x(i,:) = xpre + kf*(y - H*xpre);   % Updated state estimate
end

%% Plotting colors
blue        = '#0051ff';
light_blue  = '#b3cbff';
red         = '#ff0000';
light_red   = '#ffb3b3';
green       = '#06ad00';
light_green = '#9de69a';
black       = '#000000';


%% 1D Velocity vx, vy, vz
figure('Name', 'Velocity', 'Position', [10 10 1200 400])
subplot(1,2,1)
hold on
plot( time, x(:,1), 'LineWidth', 1.5, 'Color', red);
plot( time, x_vicon, '--', 'LineWidth', 1.5, 'Color', black  );
hold off
grid on
ylim([-1.5 1.5]);
xlabel("Time [s]");
ylabel("x [m]");
legend(["Kalman", "Ground truth"], 'location', 'southeast');
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 13)

subplot(1,2,2)
hold on
plot( time, x(:,2), 'LineWidth', 1.5, 'Color', blue);
plot( time, y_vicon, '--', 'LineWidth', 1.5, 'Color', black);
hold off
grid on
ylim([-1.5 1.5]);
xlabel("Time [s]");
ylabel("y [m]");
legend(["Kalman", "Ground truth"], 'location', 'southeast');
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 13)


%% 3D Positon plot
figure(3)
hold on
% plot( x_t2(:,1), x_t2(:,2), 'LineWidth', 1.5, 'Color', red );
plot( x(:,1), x(:,2), 'LineWidth', 1.5, 'Color', blue );

plot( x_vicon, y_vicon, '--', 'LineWidth', 1.5, 'Color', black );
legend("Kalman", "Ground Truth");
xlabel("x [m]");
ylabel("y [m]");
xlim([-1 1]);
ylim([-1 1]);
% zlim([0 1]);
yticks(-1:.5:1);
axis equal
grid on
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 13)
set(gcf, 'Position', [100, 100, 800, 800])


%% Functions

function matrix_to_cpp( matrix )

    name = inputname(1);
    [m, n] = size(matrix);
    
    tol = 1.e-6;
    matrix(matrix<=0 & matrix>-tol) = 0;
    
    fprintf('Matrix %s \n', name);
    
    for i = 1:m
        line = string();
        for j = 1:n
            str = sprintf('%.6f,', round( matrix(i,j), 6 ) );
            value = pad(str, 10, 'left');

            line = append( line, value );
        end
        disp(line);
    end
end


function world = rotate_to_world( body, p, q, u)
    R = [cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ;
         cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ;
         -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                     ];

    world = R * body;
end