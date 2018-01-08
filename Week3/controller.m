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

%u1 = 0;
%u2 = 0;

%phi_c = 0;

% FILL IN YOUR CODE HERE

% u1
K_vz = 20;
K_pz = 700;

z_dotdot = des_state.acc(2);
z_des_dot = des_state.vel(2)-state.vel(2);
z_des = des_state.pos(2)-state.pos(2);

u1 = params.mass*(params.gravity+z_dotdot+K_vz*z_des_dot+K_pz*z_des);

% phi_c
K_vy = 10;
K_py = 30;

y_dotdot = des_state.acc(1,1);
y_des_dot = des_state.vel(1,1) - state.vel(1,1);
y_des = des_state.pos(1,1) - state.pos(1,1);

phi_c = -(1/params.gravity)*(y_dotdot+K_vy*y_des_dot+K_py*y_des);

% u2 
K_vphi = 20;
K_pphi = 800;

u2 = params.Ixx*(0 + K_vphi*(0 - state.omega) + K_pphi * (phi_c - state.rot));

end

