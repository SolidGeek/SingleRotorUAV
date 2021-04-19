function [sys,x0,str,ts] = nonlinear_function_call(t,x,u,flag)

switch flag
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    s = simsizes;
        s.NumContStates  = 12;  % Number of states
        s.NumDiscStates  = 0;   % Number of discrete states
        s.NumOutputs     = 12;  % Number of outputs
        s.NumInputs      = 5;   % Number of inputs
        s.DirFeedthrough = 0;
        s.NumSampleTimes = 1;

    sys = simsizes(s);  % Initial system
    x0  = zeros(12,1);   % Initial conditions
    str = [];           % State ordering strings
    ts  = [0 0];        % An m-by-2 matrix containing the sample time (period, offset) information

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys = nonlinear_dynamics(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys = x; % The outputs are just the states

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 }
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end