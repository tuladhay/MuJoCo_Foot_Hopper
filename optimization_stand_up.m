clear
close all

s = SLIP(0);

% Just to visualize the initial state, pass SLIP(1)
% 
% init_state = s.blank_state();
% init_state.q = [0 0.5125 0 0 0 0 0];
% s.set_state(init_state);

% for i = 1:500
%    s.step();
%    
%    if mod(i,1) == 0
%        s.draw();
%    end
%    
% end
% s.close();



%minimize the simulation time
time_min = @(x) x(1);

% Initial parameter guess
% time[1],  
x0 = zeros((s.nQ + s.nQ +s.nU)*s.nodes + 1, 1);
x0(1,1) = 1.0; %time guess
% for i =(2*s.nQ*s.nodes + 2):length(x0)
%     x0(i) = 1;
% end

% x0 = x;

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
lb = [0.5;    repmat(q_lb', s.nodes, 1); ones(s.nodes * s.nQ, 1) * -Inf; repmat(u_lb', s.nodes, 1)];
ub = [Inf;  repmat(q_ub', s.nodes, 1); ones(s.nodes * s.nQ, 1) * Inf; repmat(u_ub', s.nodes, 1)];

% % Options for fmincon
% options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
%                        'MaxFunEvals', 100000, 'Display', 'iter', ...
%                        'DiffMinChange', 0.001);%, 'Algorithm', 'sqp');

options = optimoptions('fmincon','Display','iter','MaxFunEvals',10000);

% options=optimset('disp','iter','LargeScale','off','TolFun',.001,'MaxIter',100000,'MaxFunEvals',100000);
% Solve for the best simulation time + control input
x = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, constraint_func, options);





%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                                  Reference                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% INDEX
% (1) rootx
% (2) rootz
% (3) rot
% (4) left leg tau
% (5) left leg motor
% (6) left spring
% (7) left ankle
