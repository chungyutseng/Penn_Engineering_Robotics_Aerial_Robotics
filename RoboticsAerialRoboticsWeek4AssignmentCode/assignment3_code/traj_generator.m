function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 w_x w_y w_z coffx coffy coffz
if nargin > 2
    w_x = waypoints(1, :);
    w_y = waypoints(2, :);
    w_z = waypoints(3, :);
    [coffx, ~, ~] = getCoff(waypoints(1, :));
    [coffy, ~, ~] = getCoff(waypoints(2, :));
    [coffz, ~, ~] = getCoff(waypoints(3, :));
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
        scale = t/d0(t_index-1);
        coff_index = 1 + ((t_index-1) - 1) * 8 : 8 + ((t_index-1) - 1) * 8;
        t0 = polyT(8, 0, scale);
        t1 = polyT(8, 1, scale);
        t2 = polyT(8, 2, scale);
        desired_state.pos = [coffx(coff_index)' * t0'; coffy(coff_index)' * t0'; coffz(coff_index)' * t0'];
        desired_state.vel = [coffx(coff_index)' * t1' * (1 / d0(t_index-1)); coffy(coff_index)' * t1' * (1 / d0(t_index-1)); coffz(coff_index)' * t1' * (1 / d0(t_index-1))];
        desired_state.acc = [coffx(coff_index)' * t2' * (1 / d0(t_index-1) ^ 2); coffy(coff_index)' * t2' * (1 / d0(t_index-1) ^ 2); coffz(coff_index)' * t2' * (1 / d0(t_index-1) ^ 2)];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
end
%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end
%%
% function [T] = polyT(n, k, t)
%     % n is the number of unknowns
%     % k is how many times to differentiate
%     % t is the time
%     T = zeros(n, 1);
%     D = zeros(n, 1);
% 
%     %initialization
%     for i = 1 : 1 : n
%         T(i) = 1;
%         D(i) = i - 1;
%     end
%     
%     if (k == 0)
%         for i = 1 : 1 : n
%             T(i) = T(i) * (t ^ D(i));
%         end
%     else
%         for i = 1 : 1 : k
%             for j = 1 : 1 : n
%                 T(j) = T(j) * D(j);
%                 if (D(j) > 0)
%                     D(j) = D(j) - 1;
%                 end
%             end
%         end
%         for i = 1 : 1 : n 
%             T(i) = T(i) * (t ^ D(i));
%         end
%     end
%     T = T';
% end
%%