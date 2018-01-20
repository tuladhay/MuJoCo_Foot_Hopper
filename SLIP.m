classdef SLIP < handle
    %SLIP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s       %slip_t pointer
        v       %slip_vis_t pointer
        st      %state_t pointer
        nQ = 6;
        nU = 3;
        libName;
        deltaT = 0.002;
        
        flight_time;
    end
    
    
    methods
        
        %Load the SLIP library and initialize the model and visual
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
        
        
        % THIS CONTROLLER CALLS THE RAIBERT STYLE CONTROLLER IN SLIP.C
        function controller(obj)
            calllib(obj.libName, 'controller', obj.s, obj.st);
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
            unloadlibrary(obj.libName);
        end
    end
    
end

