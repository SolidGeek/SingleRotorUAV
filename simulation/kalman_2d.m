% First run record.m, to sample data from UDP
% Next run this 

load('perfect_kalman_with_vicon_02_05_2021')


%% Data Import
x_vicon = [data.xe];
y_vicon = [data.ye];
z_vicon = [data.ze];

x_vicon = x_vicon - x_vicon(1);
y_vicon = y_vicon - y_vicon(1);
z_vicon = z_vicon - z_vicon(1);

x_est = [data.xt];
y_est = [data.yt];
z_est = [data.zt];

roll = [data.roll];
pitch = [data.pitch];
yaw = [data.yaw];

% Measured lidar data
z = [data.z];

% Measured flow data (rotated with yaw)
vx = [data.vx]*1.1;
vy = [data.vy]*1.1;

% Estimated velocity
vx_est = [data.vxt];
vy_est = [data.vyt];

% Measure acceleration (rotated to body frame)
ax = [data.ax];
ay = [data.ay];
az = [data.az];


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

% Calculate discrete steady-state kalman gain

% Process Noice Covariance
sigma_acc = 0.25; % Variance of acceleration m/s^2
% Q = [dt^4/4  0       0       dt^3/2  0       0      ;
%      0       dt^4/4  0       0       dt^3/2  0      ;
%      0       0       dt^4/4  0       0       dt^3/2 ;
%      dt^3/2  0       0       dt^2    0       0      ;
%      0       dt^3/2  0       0       dt^2    0      ;
%      0       0       dt^3/2  0       0       dt^2   ] * sigma_acc;

Q = [ 1e-5   0     0      0     0     0     ;  % x
      0     1e-5   0      0     0     0     ;  % y 
      0     0     1e-5  0     0     0     ;  % z 
      0     0     0      1e-4   0     0     ;  % vx
      0     0     0      0     1e-4   0     ;  % vy
      0     0     0      0     0     5e-3 ];  % vz

% Measurement Noise Covariance
% R = [10^-5  0      0        0        0        0 ;
%      0      10^-5  0        0        0        0 ;
%      0      0      10^-6    0        0        0 ; % Lidar
%      0      0      0        2*10^-3  0        0 ; % Flow x
%      0      0      0        0        2*10^-3  0 ; % Flow y
%      0      0      0        0        0        1 ];

R = [1e-5    0      0       0       0      0 ;
     0      1e-5    0       0       0      0 ;
     0      0      1e-4    0       0      0 ; % Lidar
     0      0      0       5e-3    0      0 ; % Flow x
     0      0      0       0       5e-3    0 ; % Flow y
     0      0      0       0       0      1 ];

G = eye(6)*1;
kf = dlqe(A,G,C,Q,R);
kf = round( kf, 5 );

matrix_to_cpp(kf)

%% Run Kalman Filter

n = length(data);
x = zeros(n, 6); % State vector [ z vx vy vz ]

x(1,:) = [ 0; 0; z(1); 0; 0; 0 ];

for i = 2:n
    
    p = [0; 0; z(i)];
    p = rotate_to_world( p, roll(i), pitch(i), yaw(i) );
    h = p(3); % Height

    a = [ ax(i); ay(i); az(i)];
    u = rotate_to_world( a, roll(i), pitch(i), yaw(i) );
    %     u = [0; 0; 0];
    y = [ 0; 0; 0 ; 0; 0; 0 ];
    
    H = zeros(6,6);

%     % Vicon at 3.3 Hz
    if( data(i).stat_pos == 1 )
        H(1:2, 1:2) = eye(2);
        y(1) = x_vicon(i);
        y(2) = y_vicon(i);
        % disp('Vicon Data');
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
%     
    xpre = A*x(i-1,:)' + B*u;             % Predicted state estimate
    x(i,:) = xpre + kf*(y - H*xpre);   % Updated state estimate
end

%% Plotting

blue        = '#0051ff';
light_blue  = '#b3cbff';
red         = '#ff0000';
light_red   = '#ffb3b3';
green       = '#06ad00';
light_green = '#9de69a';
black       = '#000000';

figure('Name', 'Velocity', 'Position', [10 10 1000 400])
subplot(1,3,1)
hold on
plot( vx, 'Color', light_red  );
plot( x(:,4), 'LineWidth', 1.5, 'Color', red);
hold off
grid on
ylim([-2 2]);
title("Velocity (x-axis)");
legend("Flow", "Kalman");

subplot(1,3,2)
hold on
plot( vy, 'Color', light_blue);
plot( x(:,5), 'LineWidth', 1.5, 'Color', blue);
hold off
grid on
ylim([-2 2]);
title("Velocity (y-axis)");
legend("Flow", "Kalman");

subplot(1,3,3)
hold on
plot( x(:,6), 'LineWidth', 1.5, 'Color', green);
hold off
grid on
ylim([-2 2]);
title("Velocity (z-axis)");
legend("Kalman");

figure(2)
subplot(1,2,1);
hold on
plot( x_vicon, y_vicon );
plot( x(:,1), x(:,2) );
legend("Vicon", "Kalman");
hold off

subplot(1,2,2);
hold on
plot( z_vicon );
plot( x(:,3) );
plot( z );
legend("Vicon", "Kalman", "Measured");
hold off


figure(3)
hold on
plot3( x_vicon, y_vicon, z_vicon );
plot3( x(:,1), x(:,2), x(:,3) );
% plot3( x_est, y_est, z_est );
title("Position (world)");
legend("Vicon", "Kalman");
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
view(3);
axis equal
grid on


% figure(4)
% hold on
% plot( ax );
% plot( ay );
% plot( az );
% legend("ax", "ay", "az");
% hold off


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