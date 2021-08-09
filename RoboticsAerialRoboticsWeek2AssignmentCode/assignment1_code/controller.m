function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;
% Kp = 32;
% Kv = 6.655;
Kp = 300;
Kv = 30;
% comp = [params.mass * (Kp * (s_des(1) - s(1)) + Kv * (s_des(2) - s(2)) + params.gravity) params.u_max];
% u = min(comp);
% FILL IN YOUR CODE HERE
u = params.mass * (Kp * (s_des(1) - s(1)) + Kv * (s_des(2) - s(2)) + params.gravity);

end

