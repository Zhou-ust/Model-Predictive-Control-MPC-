function dxdt = pendulumCT0(x, u)
%% Continuous-time nonlinear dynamic model of a pendulum on a cart
%
% 4 states (x): 
%   cart position (z)
%   cart velocity (z_dot): when positive, cart moves to right
%   angle (theta): when 0, pendulum is at upright position
%   angular velocity (theta_dot): anti-clockwise positive
% 
% 1 inputs: (u)
%   force (F): when positive, force pushes cart to right 
%
% Copyright 2018 The MathWorks, Inc.

%#codegen

%% parameters
M = 1;      % cart mass
m = 1;      % pendulum mass
g = 9.81;   % gravity of earth
l = 0.5;    % pendulum length
Kd = 10;    % cart damping

%% Obtain x, u and y

% x (state variables)
z_dot = x(2);
theta = x(3);
theta_dot = x(4);

% u (input variable)
F = u;

%% Compute dxdt
dxdt = [

    z_dot;...

    (   F - Kd*z_dot ...
        - m*l*theta_dot^2*sin(theta) ...
        + m*g*sin(theta)*cos(theta)  ...
    )/(M + m*sin(theta)^2);...
    
    theta_dot;...

    (  g*sin(theta) + ...
       (F - Kd*z_dot - m*l*theta_dot^2*sin(theta))*cos(theta)/(M + m) ...
    )/(l - m*l*cos(theta)^2/(M + m));
          
   ];
