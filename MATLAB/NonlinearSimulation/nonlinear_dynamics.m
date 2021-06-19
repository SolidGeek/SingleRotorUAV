function dx = nonlinear_dynamics(t, x, u)
    
    % States
    p = x(1); q = x(2); c = x(3); wx = x(4); wy = x(5); wz = x(6); 
    px = x(7); py = x(8); pz = x(9); vx = x(10); vy = x(11); vz = x(12); 
    
    % Input
    a1 = u(1); a2 = u(2); a3 = u(3); a4 = u(4); wt = u(5);
    
    % Constants
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
    
    % Include motor model
    % Include nonlinear lift and drag coefficients
    
    % Nonlinear dynamics
    dp = wx + (wz*cos(p)*sin(q))/cos(q) + (wy*sin(p)*sin(q))/cos(q);
    dq = wy*cos(p) - wz*sin(p);
    dc = (wz*cos(p))/cos(q) + (wy*sin(p))/cos(q);
    dwx = (l*(CL*Kt*a1*wt^2 + CL*Kt*a3*wt^2) + Jy*wy*wz - Jz*wy*wz)/Jx;
    dwy = -(l*(CL*Kt*a2*wt^2 + CL*Kt*a4*wt^2) + Jx*wx*wz - Jz*wx*wz)/Jy;
    dwz = (r*(CL*Kt*a1*wt^2 - CL*Kt*a2*wt^2 - CL*Kt*a3*wt^2 + CL*Kt*a4*wt^2) + Jx*wx*wy - Jy*wx*wy)/Jz;
%     dpx = vx;
%     dpy = vy;
%     dpz = vz;
%     dvx = ((sin(p)*sin(c) + cos(p)*cos(c)*sin(q))*(Kt*wt^2 - CD*Kt*wt^2))/m - ((cos(p)*sin(c) - cos(c)*sin(p)*sin(q))*(CL*Kt*a1*wt^2 + CL*Kt*a3*wt^2))/m + (cos(q)*cos(c)*(CL*Kt*a2*wt^2 + CL*Kt*a4*wt^2))/m;
%     dvy = ((cos(p)*cos(c) + sin(p)*sin(q)*sin(c))*(CL*Kt*a1*wt^2 + CL*Kt*a3*wt^2))/m - ((cos(c)*sin(p) - cos(p)*sin(q)*sin(c))*(Kt*wt^2 - CD*Kt*wt^2))/m + (cos(q)*sin(c)*(CL*Kt*a2*wt^2 + CL*Kt*a4*wt^2))/m;
%     dvz = (cos(p)*cos(q)*(Kt*wt^2 - CD*Kt*wt^2))/m - (sin(q)*(CL*Kt*a2*wt^2 + CL*Kt*a4*wt^2))/m - g*m + (cos(q)*sin(p)*(CL*Kt*a1*wt^2 + CL*Kt*a3*wt^2))/m;
    
    dpx = vz*(sin(p)*sin(c) + cos(p)*cos(c)*sin(q)) - vy*(cos(p)*sin(c) - cos(c)*sin(p)*sin(q)) + vx*cos(q)*cos(c);
    dpy = vy*(cos(p)*cos(c) + sin(p)*sin(q)*sin(c)) - vz*(cos(c)*sin(p) - cos(p)*sin(q)*sin(c)) + vx*cos(q)*sin(c);
    dpz = vz*cos(p)*cos(q) - vx*sin(q) + vy*cos(q)*sin(p);
    dvx = (g*m*sin(q) + CL*Kt*a2*wt^2 + CL*Kt*a4*wt^2)/m;
    dvy = (CL*Kt*a1*wt^2 + CL*Kt*a3*wt^2 - g*m*cos(q)*sin(p))/m;
    dvz = -(CD*Kt*wt^2 - Kt*wt^2 + g*m*cos(p)*cos(q))/m;

    % If the total force in the body-z direction is larger then gravity (dvz > 0), or the drone already is in
    % the air (pz > 0), return all derivatives. 
    if( dvz > 0 || pz > 0 )
        % Return the derivatives
        dx = [dp; dq; dc; dwx; dwy; dwz; dpx; dpy; dpz; dvx; dvy; dvz];
    else
        % Return the derivatives
        dx = [dp; dq; dc; dwx; dwy; dwz; 0; 0; 0; 0; 0; 0];
    end
    

end