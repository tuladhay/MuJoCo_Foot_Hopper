function [c, ceq] = optimization_constraints(x, s)
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / nodes;
    
    % ********************************************************************
    % ************ Unpack T, Q, Qd and U from the x vector ***************
    % Should be a better way to do this
    pos = x(2 : (s.nQ*nodes) + 1);
    % first index is time, so shift 1 right
    pos = reshape(pos, s.nQ, nodes)';
    
    vel = x( (s.nQ*nodes + 2) : 2*s.nQ*nodes + 1 );
    % (s.nQ*nodes + 2), shift 1 for time, skip another 1 to start from one
    % index right of the previous (which in this case are the pos)
    vel = reshape(vel, s.nQ, nodes)';
    
    u = x( (2*s.nQ*nodes + 2) : length(x) );
    u = reshape(u, s.nU, nodes)';
    
    
    % ********************************************************************
    % ********** Constrain Initial Positions and Velocities **************
    c = [];
    ceq = [START CONSTRAINTS]
    
    
    for i = 1 : (nodes - 1)      

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
        
        ceq = [????????????????];
    end
    
    ceq = [ceq; END CONSTRAINTS]
    end