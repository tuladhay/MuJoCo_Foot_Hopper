% IMPLEMENT TRAJECTORY OPTIMIZATION FOR SLIP FOOT HOPPER
% START STATE AS CROUCHED, AND END STATE AS STANDING

clear
close all

target_height = 1.0;
start_height = 0.0;
N = 50;
s = SLIP(0);

% INDEX
% (1) rootx
% (2) rootz
% (3) rot
% (4) left leg tau
% (5) left leg motor
% (6) left spring
% (7) right leg tau
% (8) right leg motor
% (9) right pring

state = s.blank_state();
state.q(1) = 0;                 % root X
state.q(2) = target_height;     % root Z
state.q(3) = 0;                 % rot
state.q(4) = 0;                 % leg_tau
state.q(5) = -0.2;              % left_leg_motor range='-0.4 0.0'
state.qd(1) = 0;
state.qd(2) = 0;

% Linear inequality or equality constraints
A = [];
B = [];
