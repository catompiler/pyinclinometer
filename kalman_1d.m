clear all;
clc;

x0 = [0; 0];  % initial state (location and velocity)
P0 = [1000 0; 0 1000];  % initial uncertainty

global u = [0; 0];  % external motion

% motion model
% x' = 1 * x + t * v
% v' = 0 * x + 1 * v
% apply t = 1
global F = [1 1; 0 1];  % next state function
global H = [1 0];  % measurement function

global Id = eye(2);  % identity matrix

global R = [1];  % measurement uncertainty

function [ret_x, ret_P] = kalman_update (Z, x, P)
  global H;
  global R;
  global Id;
  
  y = Z - H * x;  % y is a residual
  S = (H * P * H') + R;
  K = P * H' * inv(S);
  
  ret_x = x + K * y;
  ret_P = (Id - K * H) * P;
endfunction

function [ret_x, ret_P] = kalman_prediction(x, P)
  global F;
  global u;
  
  ret_x = F * x + u;
  ret_P = F * P * F';
endfunction

function [ret_x, ret_P] = kalman_filter (Z, x, P)
  [x_U, P_U] = kalman_update(Z, x, P);
  [ret_x, ret_P] = kalman_prediction(x_U, P_U);
endfunction

function [x, v] = kalman_evaluate(measurements, x0, P0)
  x = [];
  v = [];
  
  x_i = x0;
  P_i = P0;
  for Z = measurements
    [x_i, P_i] = kalman_filter([Z], x_i, P_i);
    x(end + 1) = x_i(1);
    v(end +1) = x_i(2);
  endfor
endfunction

measurements = [1 2 3 4];
[x, v] = kalman_evaluate(measurements, x0, P0)
