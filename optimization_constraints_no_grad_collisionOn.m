% function [c, ceq, g_c, g_ceq] = optimization_constraints(x, s)
function [c, ceq] = optimization_constraints_no_grad_collisionOn(x, s)

    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / s.nodes;
    
%     final_state = [0.000000 0.853970 -0.000000 0.000026 -0.374286 0.001155 0.000026];

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
    ceq = [pos(1,2) - 0.704951; pos(1,5)-(-0.225268); vel(1,:)'];
    
    
    % ********** Continuous Dynamic constraints and gradients *************
    for i = 1 : (s.nodes-1)      

        state = s.blank_state();
        % States at the beginning of particular time interval
        state.q = pos(i,:)';
        state.qd = vel(i,:)';
        state.u = u(i,:)';
        
        next_state = s.blank_state();
        next_state.q = pos(i+1, :)';
        next_state.qd = vel(i+1, :)';
        next_state.u = u(i+1, :)';
        
%         qdd = s.get_qdd(state, delta_time);
        [qdd, cvel] = s.get_mj_qacc(state, delta_time);
%         % Evernote Trajectory Optimization
        proj_v = vel(i,:)' + qdd.*delta_time;
        proj_x = pos(i,:)' + proj_v.*delta_time;
        
        
        % fmincon's equality constraints are given as a vector of values to drive to zero
        % also MOVE this to "point constraints"
        ceq = [ceq ; pos(i+1,:)' - proj_x;  vel(i+1,:)' - proj_v; cvel];
        

        % To debug
        % [c, ceq] = constraint_func(x);
        % plot(ceq)
    end
    
    ceq = [ceq; pos(s.nodes, 2)-0.8325; vel(s.nodes,:)'; cvel];
    
end % end of optimization contstraints

