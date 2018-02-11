classdef SLIP < handle
    %SLIP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s       %slip_t pointer
        v       %slip_vis_t pointer
        st      %state_t pointer
        eom     %eom_fields pointer
        nQ = 7;
        nU = 3;
        nC = 2;
        libName;
        deltaT = 0.002;
        nodes = 50;

    end
    
    
    methods
        
        % ****************** General Essential Functions *****************
        % ****************************************************************
        
        function obj = SLIP(enableVisuals)
            if ispc()
                obj.libName = 'slip';
            else
                obj.libName = 'libslip';
            end
            
            if libisloaded(obj.libName)
                unloadlibrary(obj.libName)
            end
            loadlibrary(obj.libName,'src/slip.h');
            
            obj.s = calllib(obj.libName,'init','.');
            obj.st = libpointer('state_t',blank_state(obj));

            obj.eom = libpointer('EoM_fields', blank_eom_fields(obj));
            
            obj.v = [];
            if(enableVisuals == 1)
                obj.v = calllib(obj.libName, 'vis_init');
            end
        end   
        
   
        function set_motor_command(obj,u)
           obj.st.Value.u(1) = u(1); 
           obj.st.Value.u(2) = u(2);
           obj.st.Value.u(3) = u(3);
        end
        
        
        function step(obj)
            calllib(obj.libName, 'step', obj.s, obj.st);
        end
                
        
        function state = get_state(obj)
            state = obj.st.Value;
        end
        
            
        function set_state(obj,state)
            obj.st.Value.q = state.q;
            obj.st.Value.qd = state.qd;
            obj.st.Value.qdd = state.qdd;
            obj.st.Value.t = state.t;
            obj.st.Value.dynamic_state = state.dynamic_state;
            obj.st.Value.des_td_angle = state.des_td_angle;
            obj.st.Value.touchdown_time = state.touchdown_time;
            obj.st.Value.stance_time = state.stance_time;
            obj.st.Value.apex_velocity = state.apex_velocity;
            calllib(obj.libName, 'set_state', obj.s, obj.st);
            % Also check set_state in slip.c
        end
        

        function state = blank_state(obj)
           state.q = zeros(1, obj.nQ);
           state.qd = state.q;
           state.qdd = state.q;
           state.u = zeros(1, obj.nU);
           state.t = 0;
           state.dynamic_state = 3;     % start in flight mode
           state.des_td_angle = 0;
           state.touchdown_time = 0;
           state.stance_time = 0;
           state.apex_velocity = 0;
        end
        
        
        function state = run_forward(obj, s, h)
           s = libpointer('state_t', s);
           calllib('libslip', 'run_forward', obj.s, s, h);
           
           state = s.Value;
        end
        % **************** Optimization Related Functions ****************
        % ****************************************************************
        
        function eom_fields = blank_eom_fields(obj)
            eom_fields.H = zeros(1, obj.nQ*obj.nQ);
            eom_fields.h = zeros(1, obj.nQ);
            eom_fields.J = zeros(1, 6*obj.nQ);
            eom_fields.Jdot_Qdot = zeros(1, 3*obj.nC);
        end
        
        
        function eom_fields = get_eom(obj, state)
            calllib(obj.libName, 'get_EoM_fields', obj.s, state, obj.eom);
            eom_fields = obj.eom.Value;
        end
        
        
        function qdd = get_qdd(obj, state)
            % Function to get necessary quantities from MuJoCo,
            % and calculate qdd and return qdd
            eom_copy = obj.get_eom(state);   % I know it is redundant to make copy
            
            H = reshape(eom_copy.H, [obj.nQ, obj.nQ]);
            if (H - H')
                disp('Mass matrix not symmetric');
            end

            % Get the h matrix for coriolis, centripetal, gravity and spring terms
            h = reshape(eom_copy.h, [obj.nQ, 1]);
            % need to make sure this is correct

            % Get the contact Jacobian
            J = reshape(eom_copy.J, [obj.nQ, 6])';  %Transpose is correct?            
            
            % need to make sure this is correct
            % the rows 1, 2, 3 are for contact site 0
            % the rows 4, 5, 6 are for contact site 1

            % J*Qdd + Jdot*Qdot = xdd, set qdd to zero, and get JdotQdot = xdd
            Jdot_Qdot = reshape(eom_copy.Jdot_Qdot, [6, 1]);

            % These equations are directly from Wensing paper:
            % "Generation of Dynamic Humanoid Behaviors Through Task-Space Control ..."

            Tau = state.u;     % Torques
            Sa = [0; 0; 0; Tau(1); Tau(2); 0; Tau(3)]; % Selector matrix premultiplied by Tau
            I = eye(obj.nQ);
            Hinv = inv(H);          % Is this the correct way to take inverse?
            JHinvJT = J*Hinv*J';   % How to do pseudo-inverse?
            Ns = I - J'*pinv(JHinvJT)*J*Hinv;
            gamma = J'*pinv(JHinvJT)*Jdot_Qdot;

            qdd = Hinv*(Ns*Sa - Ns*h - gamma);      % maybe group Ns
            
            % JUST TRYING TO SEE WHAT HAPPENS !!!!2342342@$^3457!!!!!!!!!!(&#(&#(&#(#&(#
            %qdd = eom_copy.qacc;
            
            % *************** Calculating f just to see *******************
            site_force = pinv(JHinvJT)*( J*Hinv*h - Jdot_Qdot - J*Hinv*Sa ); % on site
            calc_qfrc = J'*site_force;   % Fx = J'*Fq. See studywolf.
        end
        
        function mj_qdd = get_mj_qacc(obj, state)
            eom_copy = obj.get_eom(state);
            mj_qdd = eom_copy.qacc';
        end
        
        
        function [lb, ub] = get_state_limits(obj)
            bounds.lb = zeros(1, obj.nQ);
            bounds.ub = zeros(1,obj.nQ);
            bounds = libpointer('pos_limits_t', bounds);
            calllib('libslip', 'get_joint_limits', bounds);
            lb = bounds.Value.lb;
            ub = bounds.Value.ub;
        end
        
        
        function [lb, ub] = get_motor_limits(obj)
            bounds.lb = zeros(1, obj.nU);
            bounds.ub = zeros(1,obj.nU);
            bounds = libpointer('motor_limits_t', bounds);
            calllib('libslip', 'get_motor_limits', bounds);
            lb = bounds.Value.lb;
            ub = bounds.Value.ub;
        end
        
        
            
        % ***************** Raibert Controller Function ******************
        % ****************************************************************
                
        function controller(obj)
            calllib(obj.libName, 'controller', obj.s, obj.st);
        end
         
       
        % ************* Rendering and Close Functions ********************
        % ****************************************************************
                
        function rendered = draw(obj)
            if ~isempty(obj.v)
                rendered = calllib(obj.libName, 'vis_draw', obj.v, obj.s,0);
            end
        end
        
        function close(obj)
            if ~isempty(obj.v)
                calllib(obj.libName,'vis_close',obj.v)
            end
            
            obj.s = [];
            obj.v = [];
            obj.st = [];
            obj.eom = [];
            unloadlibrary(obj.libName);
        end
    end
    
end

