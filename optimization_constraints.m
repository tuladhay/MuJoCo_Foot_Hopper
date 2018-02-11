function [c, ceq, g_c, g_ceq] = optimization_constraints(x, s)

    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / s.nodes;
    eps = 1e-6;
    
    final_state = [0.000000 0.853970 -0.000000 0.000026 -0.374286 0.001155 0.000026];

    % *********************************************************************
    % ************ Unpack T, Q, Qd and U from the x vector ****************
    % Should be a better way to do this
    pos = x(2 : (s.nQ*s.nodes) + 1);
    % first index is time, so shift 1 right
    pos = reshape(pos, s.nQ, s.nodes)';
    
    vel = x( (s.nQ*s.nodes + 2) : 2*s.nQ*s.nodes + 1 );
    % (s.nQ*s.nodes + 2), shift 1 for time, skip another 1 to start from one
    % index right of the previous (which in this case are the pos)
    vel = reshape(vel, s.nQ, s.nodes)';
    
    u = x( (2*s.nQ*s.nodes + 2) : length(x) );
    u = reshape(u, s.nU, s.nodes)';
    % *********************************************************************
    
    c = [];
    ceq = [];
    g_c = [];
    g_ceq = [];
    temp_grad = [];
    
    % replace this by a function which calculates start constraints and
    % start constraint gradients
    ceq_start = [pos(1,2) - 0.70495; vel(1,:)']; % constraint on initial rootz and q vels
    
    
    % *********************************************************************
    % Constrains and gradients for initial position and velocity ceq and ceq grad
    ceq = [ceq; ceq_start];
    
    temp_grad = zeros(length(x),length(ceq_start));
    
    temp_grad(3,1) = 1;     % gradient of x2 (i.e., zpos wrt x2 is 1)
    qd_idx = (1 + (s.nQ)*s.nodes : s.nQ + (s.nQ)*s.nodes) + 1;
    % ROWS: Increment 1 to skip time. Increment +1 to skip rootx.
    
    temp_grad(qd_idx, [2 3 4 5 6 7 8]) = eye(7); % better ways to do this?
    
    % THE START TIME GRADIENTS ARE AUTOMATICALLY ZERO
    
    g_ceq = [g_ceq, temp_grad];
    % *********************************************************************
    
    % ********** Continuous Dynamic constraints and gradients *************
    for i = 2 : (s.nodes - 1)      

        state = s.blank_state();
        % States at the beginning of particular time interval
        state.q = pos(i,:)';
        state.qd = vel(i,:)';
        state.u = u(i,:)';
        
        next_state = s.blank_state();
        next_state.q = pos(i+1, :)';
        next_state.qd = vel(i+1, :)';
        next_state.u = u(i+1, :)';
        
%         % Move this to separate function "point constraints"
%         qdd = s.get_qdd(state);
%         % Evernote Trajectory Optimization
%         proj_v = vel(i,:)' + qdd.*delta_time;
%         proj_x = pos(i,:)' + proj_v.*delta_time;
        
        % and only call this function
        [ceq_n, ceq_ngrad] = constraints_and_gradient(state, next_state, s, delta_time);
%         ceq_ngrad = ceq_ngrad';
        % fmincon's equality constraints are given as a vector of values to drive to zero
        % also MOVE this to "point constraints"
%         ceq = [ceq ; pos(i+1,:)' - proj_x;  vel(i+1,:)' - proj_v];
        ceq = [ceq; ceq_n];
        
        temp_grad = zeros(length(x),length(ceq_n));
        q_idx = (s.nQ*(i-1) + 1: s.nQ*i) + 1;
        qd_idx = (s.nQ*(i-1) + (1 + (s.nQ)*s.nodes) : s.nQ*(i) + (s.nQ)*s.nodes) + 1;
        u_idx = (s.nU*(i-1) + (1 + (2*s.nQ)*s.nodes) : s.nU*(i) + (2*s.nQ)*s.nodes) + 1;
        
        % Gradients wrt next set of positions, velocities and controls
        qp_idx = (s.nQ*(i) + 1: s.nQ*(i+1)) + 1;
        qdp_idx = (s.nQ*(i) + (1 + (s.nQ)*s.nodes) : s.nQ*(i+1) + (s.nQ)*s.nodes) + 1;
        up_idx = (s.nU*(i) + (1 + (2*s.nQ)*s.nodes) : s.nU*(i+1) + (2*s.nQ)*s.nodes) + 1;
        
        time_row = 1;
        temp_grad(time_row,:) =  ceq_ngrad(time_row,:);
        
        temp_grad(q_idx, :) = ceq_ngrad((1:s.nQ) + 1, :);
        temp_grad(qd_idx, :) = ceq_ngrad((s.nQ+1 : 2*s.nQ)+1 , :);
        temp_grad(u_idx, :) = ceq_ngrad((2*s.nQ+1 : 2*s.nQ+s.nU) + 1, :);
        
        temp_grad(qp_idx,:) = ceq_ngrad((2*s.nQ+s.nU+1:3*s.nQ+s.nU) + 1,:);
        temp_grad(qdp_idx,:) = ceq_ngrad((3*s.nQ+s.nU+1:4*s.nQ+s.nU) + 1,:);
        temp_grad(up_idx,:) = ceq_ngrad((4*s.nQ+s.nU+1:4*s.nQ+2*s.nU) + 1,:);
        % THE ADDED +1 IN MOST ROWS ARE DUE TO THE OFFSET FOR INCLUDING A
        % ROW FOR TIME
        
        g_ceq = [g_ceq, temp_grad];

        % To debug
        % [c, ceq] = constraint_func(x);
        % plot(ceq)
    end
    
%     ceq = [ceq; pos(s.nodes, 2)-0.8425];% vel(s.nodes,:)'];
    
    % Get end_state constraints and gradients for end_states
    ceq_end = [(pos(s.nodes,2) - 0.85); vel(s.nodes,:)'];
    ceq = [ceq; ceq_end];
    
    temp_grad = zeros(length(x),length(ceq_end));
    % Size of start_state and end_state temp_grad(s) are different that
    % inner dynamics temp_grad since number of start/end constraints are
    % different
    
    temp_grad(s.nQ*(s.nodes-1)+3, 1) = 1;   % gradient of z pos of final node wrt final rootz
    % (s.nQ*(s.nodes-1)+3) because i want to offset (nodes-1)*nQ, since
    % we are calculating gradient for the last node. Then +1 goto next row,
    % +1 again to offset (rootx), +1 to offset time
        
    % Then we will be at rootz for the last node
    % Since the last 8 are pos; and vel[1...7].
    
    % gradient of final_state velocity wrt final node velocity
    qd_idx = (length(ceq_start)+(2*s.nQ)*(s.nodes-1) : (2*s.nQ)*(s.nodes)) + 1; 
    
    temp_grad(qd_idx, [2 3 4 5 6 7 8]) = eye(7);
    
    % has to be length of (s.nodes-1)*length(ceq_n) + length(ceq_start) +
    % length(ceq_end)
    g_ceq = [g_ceq, temp_grad];
        
end % end of optimization contstraints


function [ceq, ceq_grad] = constraints_and_gradient(state, next_state, s, delta_time)
    eps = 1e-6;
    offset = 2*s.nQ+s.nU;
    
    % ceq used for the constraint
    % Everything else is for the gradients
    [ceq, t_g_p, t_g_v] = point_constraints(state, next_state, s, delta_time);
    time_grad = [t_g_p; t_g_v]';
    
    %ceq_grad = zeros(2*s.nQ + s.nU, length(ceq));
    ceq_grad = zeros(2*(2*s.nQ + s.nU) + 1, length(ceq));
    ceq_grad(1,:) = time_grad;
    
    
    for j = 1:s.nQ    % Gradients for q in x vector
        state.q(j) = state.q(j) + eps;
        [cn, ~, ~] = point_constraints(state, next_state, s, delta_time);
        ceq_grad(j+1,:) = (cn - ceq)/(eps);
        state.q(j) = state.q(j) - eps;
        
        
        % gradients wrt next state
        next_state.q(j) = next_state.q(j) + eps;
        [cn, ~, ~] = point_constraints(state, next_state, s, delta_time);
        ceq_grad(j+offset+1,:) = (cn -ceq)/(eps);    % <<<
        next_state.q(j) = next_state.q(j) - eps;

    end

    for j = 1:s.nQ    % Gradients for qd in x vector
        state.qd(j) = state.qd(j) + eps;
        [cn, ~, ~] = point_constraints(state, next_state, s, delta_time);
        ceq_grad(j+s.nQ+1,:) = (cn - ceq)/(eps);
        state.qd(j) = state.qd(j) - eps;
        
        next_state.qd(j) = next_state.qd(j) + eps;
        [cn, ~, ~] = point_constraints(state, next_state, s, delta_time);
        ceq_grad(j+offset+s.nQ+1,:) = (cn -ceq)/(eps);    % <<<
        next_state.qd(j) = next_state.qd(j) - eps;

    end

    for j = 1:s.nU    % Gradients for u
        state.u(j) = state.u(j) + eps;
        [cn, ~, ~] = point_constraints(state, next_state, s, delta_time);
        ceq_grad(2*s.nQ+j+1,:) = (cn - ceq)/(eps);
        state.u(j) = state.u(j) - eps;
        
        next_state.u(j) = next_state.u(j) + eps;
        [cn, ~, ~] = point_constraints(state, next_state, s, delta_time);
        ceq_grad(j+offset+2*s.nQ+1,:) = (cn -ceq)/(eps);    % <<<
        next_state.u(j) = next_state.u(j) - eps;

    end

end % end ceq_grad


function [calc_ceq, time_grad_pos, time_grad_vel] = point_constraints(state, next_state, s, delta_time)
    % instead of using run_forward, i need to do this manually by getting
    % qdd and calculating proj_v and proj_x
    qdd = s.get_qdd(state);
    proj_v = state.qd + qdd.*delta_time;
    proj_x = state.q + proj_v.*delta_time;
    calc_ceq = [next_state.q - proj_x;  next_state.qd - proj_v];
    
    time_grad_pos = (-1*proj_v)/s.nodes;
    time_grad_vel = (-1*qdd)/s.nodes;
end
