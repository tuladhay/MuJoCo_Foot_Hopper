% START STATE AS CROUCHED, AND END STATE AS STANDING

clear
close all
global nodes
nodes = 20;
s = SLIP(0);

%minimize the simulation time
time_min = @(x) x(1);

% Initial parameter guess
% time[1],  
x0 = zeros((s.nQ + s.nQ +s.nU)*nodes + 1, 1);
x0(1,1) = 1.0; %time guess

% Handle for constraint function
constraint_func = @(x)optimization_constraints(x, s);

% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];

% Bounds
[q_lb, q_ub] = s.get_state_limits();
[u_lb, u_ub] = s.get_motor_limits();

% Ordered by: time | positions | velocities | control
lb = [0;    repmat(q_lb', s.N, 1); ones(s.N * s.nQ, 1) * -Inf; repmat(u_lb', s.N, 1)];
ub = [Inf;  repmat(q_ub', s.N, 1); ones(s.N * s.nQ, 1) * Inf; repmat(u_ub', s.N, 1)];

% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, 'Display', 'iter', ...
                       'DiffMinChange', 0.001, 'Algorithm', 'sqp');

% Solve for the best simulation time + control input
x = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, constraint_func, options);







%%
% s = SLIP(0);
% nStep = 100;
% for i = 1:nStep
%     qdd = s.get_qdd();
%     % this will call get_eom and calculate qdd and return it
% end
% 
% s.close();

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Optimization Setup                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% INDEX
% (1) rootx
% (2) rootz
% (3) rot
% (4) left leg tau
% (5) left leg motor
% (6) left spring
% (7) left ankle
