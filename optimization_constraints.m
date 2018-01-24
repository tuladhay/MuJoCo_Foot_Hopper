function [c, ceq] = optimization_constraints(x, s)
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / s.nodes;
    
    % ********************************************************************
    % ************ Unpack T, Q, Qd and U from the x vector ***************
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
    
    
    % ********************************************************************
    % ********** Constrain Initial Positions and Velocities **************
    c = [];
    % fmincon's equality constraints are given as a vector of values to drive to zero
    ceq = []; % NOT SURE WHAT TO PUT FOR START CONSTRAINTS
    ceq = [pos(1,2) - 0.5125]; % constraint on initial rootz
    
    for i = 1 : (s.nodes - 1)      

        state = s.blank_state();
        % States at the beginning of particular time interval
        state.q = pos(i,:)';
        state.qd = vel(i,:)';
        state.u = u(i,:)';
        
        % Get qdd
        qdd = s.get_qdd(state);
        % qdd for contact 1
        qdd_c1 = qdd(:,1);
        
        % See evernote Trajectory Optimization
        proj_v = vel(i,:)' + qdd_c1.*delta_time;
        proj_x = pos(i,:)' + proj_v.*delta_time;
        
        % fmincon's equality constraints are given as a vector of values to drive to zero
        ceq = [ceq ; pos(i+1,:)' - proj_x; vel(i+1,:)' - proj_v];
        
        % To debug
        % [c, ceq] = constraint_func(x);
        % plot(ceq)
    end
    
    % NOT SURE ABOUT THIS!
    ceq = [ceq; pos(s.nodes, 2)-0.85; vel(s.nodes,:)'];
    
    end