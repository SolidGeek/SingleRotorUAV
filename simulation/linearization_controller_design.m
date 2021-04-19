syms x y z vx vy vz p q u wx wy wz      % States
syms Ft F1 F2 F3 F4 Fd a1 a2 a3 a4 wt   % Inputs 
syms Kt CL CD CF m Jx Jy Jz r l g       % Constants

%% Definitions
% Transformation matrix from body angular velocity to Tait-Bryan rates
W = [ 1, 0,      -sin(q)         ;
      0, cos(p),  cos(q)*sin(p)  ;
      0, -sin(p), cos(q)*cos(p) ];

Winv = simplify(inv(W));
  
% Rotation matrix from body to world frame, input: roll, pitch, yaw
R = [cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ;
     cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ;
     -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                     ];
 

% Matrix of mass inertia
J = [Jx 0  0  ;
     0  Jy 0  ;
     0  0  Jz];
 
% Forces 
Ft = Kt * wt^2;
F1 = Ft * CL * a1;
F2 = Ft * CL * a2;
F3 = Ft * CL * a3;
F4 = Ft * CL * a4;
Fd = Ft * CD; 

% Body forces
fb = [ F2 + F4  ;
       F1 + F3  ;
       Ft - Fd ];

% Body torques
tb = [ (F1 + F3)*l            ;
      -(F2 + F4)*l            ;
       (F1 - F2 - F3 + F4)*r ];
   
% State vectors used for derivation
nw = [p q u].';     % Attitude (world frame)
wb = [wx wy wz].';  % Angular velocity (body frame)
pw = [x y z].';     % Position (world frame)
vb = [vx vy vz].';  % Velocity (body frame)

% Total state vector
X = [nw; wb; pw; vb];
X_red = [nw; wb; pw(3); vb(3)]; % Reduced state vector (only attitude and altitude)

% Input vector 
U = [a1; a2; a3; a4; wt];

%% Rotational dynamics

nw_dot = Winv * wb;
wb_dot = inv(J) * (tb - cross(wb, J * wb)  );

%% Translational dynamics

pw_dot = R * vb;
vb_dot = 1/m * ( fb -  R.' * [0 0 m*g].');

%% Combined non-linear model

f = [ nw_dot  ;
      wb_dot  ;
      pw_dot  ;
      vb_dot ];

% Reduced non-linear model
f_red = [ nw_dot     ;
          wb_dot     ;
          pw_dot(3)  ;
          vb_dot(3) ];
  
%% Linearization

% Using the Jacobian method, the set of nonlinear system equations are
% linearized around the hover point

A = jacobian(f, X);
B = jacobian(f, U);

% Reduced model (only z-axis in position)
A2 = jacobian(f_red, X_red);
B2 = jacobian(f_red, U);

% The A and B matrixes are now filled with partial derivatives, similar to
% an taylor expansion to approximate a nonlinear function/ODE
% We must insert the state- and input-values at the operating point

% All the states is zero at the hover point
x = 0; y = 0; z = 0; vx = 0; vy = 0; vz = 0; p = 0; q = 0; u = 0; wx = 0; wy = 0; wz = 0;

% All input is not zero!
a1 = 0; a2 = 0; a3 = 0; a4 = 0; 
wt = 20.062; % kRPM = hoverpoint

% Drone Constants
Kt = 0.021952;      % N / (1/s^2)
CL = 0.008905;      % -
CD = 0.001054;      % -
Jx = 0.01031759;    % Kg * m^2
Jy = 0.01031865;    % Kg * m^2
Jz = 0.00278832;    % Kg * m^2
m = 0.92;           % Kg
g = 9.807;          % m/s^2
l = 0.09471940;     % m
r = 0.04;           % m 

% Now the A and B matrixes can be evaluted, yield the full linear model
% around the hover point.
A_sys = double(vpa(subs(A), 4));
B_sys = double(vpa(subs(B), 4));
C_sys = eye(12);
D_sys = zeros(12,5);

% Reduced model 
A_red = double(vpa(subs(A2), 4));
B_red = double(vpa(subs(B2), 4));
C_red = eye(8);
D_red = zeros(8,5);

%% Open Loop dynamics

sys = ss(A_sys,B_sys,C_sys,D_sys);
sys_red = ss(A_red,B_red,C_red,D_red);


%% Design controller

% Bryson's Rule. 
% Max angle of 0.3 radians. Maximum angular rate of 5 rad/second
Q = [ 1/0.3^2  0        0        0      0      0      0        0       ;  % Roll
      0        1/0.3^2  0        0      0      0      0        0       ;  % Pitch
      0        0        1/0.3^2  0      0      0      0        0       ;  % Yaw
      0        0        0        1/5^2  0      0      0        0       ;  % omega_x
      0        0        0        0      1/5^2  0      0        0       ;  % omega_y
      0        0        0        0      0      1/5^2  0        0       ;  % omega_z
      0        0        0        0      0      0      1/10^2   0       ;  % z
      0        0        0        0      0      0      0        1/10^2  ]; % v_z
      
% Max actuation angle of +-10 degress
R = [ 1/10^2   0       0       0       0       ; % a1
      0        1/10^2  0       0       0       ; % a2
      0        0       1/10^2  0       0       ; % a3
      0        0       0       1/10^2  0       ; % a4
      0        0       0       0       1/50^2 ]; % wt

% Compute "optimal" controller
K_lqr = lqr(sys_red, Q, R);

% Calcuate closed loop system
cl_sys = ss((A_red - B_red*K_lqr), B_red, C_red, D_red );


%% Symbolic Discretization

% syms dt;
% M = expm([A_sym, B_sym; zeros(5,12), zeros(5,5) ]*dt);
% 
% Ad = M(1:12, 1:12);
% Bd = M(1:12, 13:17);