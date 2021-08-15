function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;
%%
Kp_y = 0; %10.4
Kd_y = 20;  %2.2
Kp_z = 400;
Kd_z = 14.285;
Kp_phi = 1600;
Kd_phi = 26.66;
%%
% Kp_y = 0; %10.4
% Kd_y = 0;  %2.2
% Kp_z = 0;
% Kd_z = 0;
% Kp_phi = 3000;
% Kd_phi = 0;
%%
u1 = params.mass * (params.gravity + des_state.acc(2) + Kd_z*(des_state.vel(2) - state.vel(2)) + Kp_z*(des_state.pos(2) - state.pos(2)));
phi_c = (-1/params.gravity) * (des_state.acc(1) + Kd_y*(des_state.vel(1) - state.vel(1)) + Kp_y*(des_state.pos(1) - state.pos(1)));
u2 = params.Ixx * (Kd_phi*(-state.omega) + Kp_phi*(phi_c - state.rot));
% phi_c = (-1/params.gravity) * (des_state.acc(1) + Kd_y*(des_state.vel(1) - state.vel(1)) + Kp_y*(des_state.pos(1) - state.pos(1)));
% u2 = params.Ixx * (des_state.acc(1) + Kd_y*(des_state.vel(1) - state.vel(1)) + Kp_y*(des_state.pos(1) - state.pos(1)));
% FILL IN YOUR CODE HERE
%% For tuning Kp_phi  & Kd_phi
% Ku_phi = 2000;
% Tu_phi = 0.1333;
% Kp_phi = 0.8 * Ku_phi;
% Kd_phi = 0.1 * Ku_phi * Tu_phi;
%% For tuning Kp_z  & Kd_z
% Ku_z = 500;
% Tu_z = 0.2857;
% Kp_z = 0.8 * Ku_z;
% Kd_z = 0.1 * Ku_z * Tu_z;
%% For tuning Kp_y  & Kd_y
% Ku_y = 13;
% Tu_y = 1.7;
% Kp_y = 0.8 * Ku_y;
% Kd_y = 0.1 * Ku_y * Tu_y;
%%
end

