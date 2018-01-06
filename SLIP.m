classdef SLIP < handle
    %SLIP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s %slip_t pointer
        v %slip_vis_t pointer
        st %state_t pointer
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
        end
        
        function step(obj)
            calllib(obj.libName, 'step', obj.s, obj.st);
        end
        
        
        function set_dynamic_state(obj)
            calllib(obj.libName, 'set_dynamic_state', obj.st);
        end
        
        
        function controller(obj)
            calllib(obj.libName, 'controller', obj.s, obj.st);
        end
        
        
        function state = get_state(obj)
            state = obj.st.Value;
        end
        
        function dynamic_state = get_dynamic_state(obj)
            dynamic_state = obj.st.Value.dynamic_state;
        end
        
        
        function des_td_angle = get_des_td_angle(obj)
            des_td_angle = obj.st.Value.des_td_angle;
        end
        
        
%         function contact_pos = get_contact_pos(obj)
%             contact_pos = obj.st.Value.cpos;
%         end
        
            
        function set_state(obj,state)
            obj.st.Value.q = state.q;
            obj.st.Value.qd = state.qd;
            obj.st.Value.qdd = state.qdd;
            obj.st.Value.t = state.t;
            
            calllib(obj.libName, 'set_state', obj.s, obj.st);
        end
        
        function state = blank_state(obj)
           state.q = zeros(1, obj.nQ);
           state.qd = state.q;
           state.qdd = state.q;
           state.u = zeros(1, obj.nU);
           state.t = 0;
        end
                
%         function [qdd] = dynamics(obj, state)
%             state = libpointer('state_t', state);
%             calllib('libslip', 'forward', obj.s, state);
%             temp = state.Value;
%             qdd = temp.qdd;           
%         end
        
%         function [lb, ub] = get_state_limits(obj)
%             bounds.lb = zeros(1, obj.nQ);
%             bounds.ub = zeros(1,obj.nQ);
%             bounds = libpointer('pos_limits_t', bounds);
%             calllib('libslip', 'get_joint_limits', bounds);
%             lb = bounds.Value.lb;
%             ub = bounds.Value.ub;
%         end
        
%         function [lb, ub] = get_motor_limits(obj)
%             bounds.lb = zeros(1, obj.nU);
%             bounds.ub = zeros(1,obj.nU);
%             bounds = libpointer('motor_limits_t', bounds);
%             calllib('libslip', 'get_motor_limits', bounds);
%             lb = bounds.Value.lb;
%             ub = bounds.Value.ub;
%         end
        
        
        
%         function state = run_forward(obj, s, h)
%            s = libpointer('state_t', s);
%            calllib('libslip', 'run_forward', obj.s, s, h); 
%            state = s.Value;
%         end
        
        
                
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

