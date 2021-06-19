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