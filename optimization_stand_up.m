clear
close all

s = SLIP(0);
s.get_state();
init_state = s.blank_state();
init_state.q = [0 0.704951 0 0 -0.225268 0.00115 0];
s.set_state(init_state);

% Just to visualize the initial state, pass SLIP(1)
% 
% init_state = s.blank_state();
% init_state.q = [0 0.5125 0 0 0 0 0];
% s.set_state(init_state);

% for i = 1:5000
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
x0(1,1) = 0.41; %time guess

% Setting rootz of x0 to 0.5
for i = 3: s.nQ :(s.nQ*s.nodes)
    x0(i,1) = 0.7050 + (0.8425 - 0.7050)*((i)/ (s.nQ*s.nodes));
    x0(i+3,1) = -0.225;
end

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
lb = [0.15;    repmat(q_lb', s.nodes, 1); ones(s.nodes * s.nQ, 1) * -Inf; repmat(u_lb', s.nodes, 1)];
ub = [Inf;  repmat(q_ub', s.nodes, 1); ones(s.nodes * s.nQ, 1) * Inf; repmat(u_ub', s.nodes, 1)];

% % Options for fmincon

% options = optimoptions('fmincon','Display','iter','MaxFunEvals',20000, ...
%     'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-6);
%options.Algorithm = 'sqp';

options = optimoptions(@fmincon,'ConstraintTolerance',1e-3,'StepTolerance',1e-4,...
    'SpecifyConstraintGradient',true,...
    'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-6,...
    'CheckGradients',true,...
    'MaxFunEvals', 10000, 'Display', 'iter');


x = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, constraint_func, options);


% below is not working yet
extracted.q1 = [];
for i = 3: s.nQ :(s.nQ*s.nodes)
    extracted.q1 = [extracted.q1; x(i,1)];
end

    
%s.close()

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
