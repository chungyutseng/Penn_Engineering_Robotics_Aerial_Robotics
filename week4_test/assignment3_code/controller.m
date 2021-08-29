function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%% PD Controller Parameter
kd_1 = 25;
kp_1 = 20;
kd_2 = 25;
kp_2 = 0;
kd_3 = 20;
kp_3 = 400;

kd_phi = 30;
kp_phi = 1400; %1500
kd_theta = 30;
kp_theta = 1400;
kd_psi = 30;
kp_psi = 1500;
%% For r_ddot_1_des, r_ddot_2_des, r_ddot_3_des
r_ddot_1_des = des_state.acc(1) + kd_1 * (des_state.vel(1) - state.vel(1)) + kp_1 * (des_state.pos(1) - state.pos(1));
r_ddot_2_des = des_state.acc(2) + kd_2 * (des_state.vel(2) - state.vel(2)) + kp_2 * (des_state.pos(2) - state.pos(2));
r_ddot_3_des = des_state.acc(3) + kd_3 * (des_state.vel(3) - state.vel(3)) + kp_3 * (des_state.pos(3) - state.pos(3));
%% Calculate the Desired Roll, Pitch, and Yaw Angle with r_ddot_1_des, r_ddot_2_des, r_ddot_3_des
% Roll(phi) Pitch(theta) Yaw(Psi)
phi_des = (1/params.gravity) * (r_ddot_1_des * sin(des_state.yaw) - r_ddot_2_des * cos(des_state.yaw));
theta_des = (1/params.gravity) * (r_ddot_1_des * cos(des_state.yaw) + r_ddot_2_des * sin(des_state.yaw));
psi_des = des_state.yaw;
%%
% Thrust
F = 0;
u1 = params.mass * params.gravity + params.mass * r_ddot_3_des; 
F = u1;
% Moment
M = zeros(3,1);
u2 = [kp_phi * (phi_des - state.rot(1)) + kd_phi * (0 - state.omega(1));
      kp_theta * (theta_des - state.rot(2)) + kd_theta * (0 - state.omega(2));
      kp_psi * (psi_des - state.rot(3)) + kd_psi * (des_state.yawdot - state.omega(3));];
M = params.I * u2;
% =================== Your code ends here ===================

end
