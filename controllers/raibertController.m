function [ u ] = raibertController(q,qdot,qddot)
%RAIBERTCONTROLLER A faithful implementation of Raibert's monopod hopping
%controller


loadingCompression = 0.05; %Amount of compression for Loading and Unloading phases (m)
liftOffClearance = 0.05; %Distance toe must rise before the leg can be swung forward during flight (m)
k_legAngle = 0.01; %m/(m/s) foot distance 
L_flight = 0.4; %unstretched leg length during compression and flight (m)
L_extension = 0.5; %unstretched leg length during thrust (m)
des_height=0.7;
des_vel = 0.3; %m/s
phi_des = 0; %Rads

%Low Level controller gains, UNTUNED
kp_swing = 500;
kv_swing = 5;
kp_hip = 100;
kv_hip = 10;
kp_L0 = 75000;
kv_L0 = 300;

%States: 1= Loading, 2= Compression, 3= Thrust, 4= Unloading, 5=Flight
persistent stateMachine footTDAngle
if isempty(stateMachine) || isempty(footTDAngle)
    stateMachine = 3; %Simulation starts by dropping robot
    footTDAngle = 0;
    disp('initialized persistents in raibert controller');
end

%Check for a change in the state 
switch stateMachine
    case 1 %Compression
        %Transition if the leg begins to extend
        %if qdot(6) >= 0
        if qddot(2) >= 0.001
            stateMachine = 2; %fine
            disp('Controller End Compression');
        end  
    case 2 %Thrust
        %Transition once the leg compression is less than the threshold
        %if q(5) - q(6) <= loadingCompression
        if (q(2) - L_flight*cos(q(3)+q(4))) > 0.05
            stateMachine = 3;
            disp('Controller End Thrust');
            
            %Calculate the new desired leg angle
            x_f0 = 0.5*qdot(1)*0.12 + k_legAngle*(qdot(1) - des_vel);
            
            x_f0 = min(max(x_f0 ,-0.7*L_flight),0.7*L_flight); %Cap the forward leg displacement to keep it reasonable
            %Desired touchdown leg angle
            footTDAngle = asin(x_f0/L_flight);
            disp(['New TD Angle ',num2str(footTDAngle)]);
        end 
    
    case 3 %Flight
        %Transition if the foot touches the ground
        if q(2) - L_flight*cos(q(3)+ q(4)) <= 0               % to fix the axis.
            stateMachine = 1;
            disp('Controller has Landed');
        end
end

%Force and Torque Controllers depending on state
switch stateMachine
    
    case 1 %Compression
        u(1) = -kp_L0*(q(5) - L_flight) - kv_L0*qdot(5);
        u(2) = kp_hip*(q(3) - phi_des) + kv_hip*qdot(3);
        
    case 2 %Thrust
        height_error= -(q(2) - des_height);
        des_mot_pos=0.4 + 2*height_error;
        
        %u(1) = -kp_L0*(q(2) - des_height);
        u(1) = -kp_L0*(q(5) - des_mot_pos);
        u(2) = kp_hip*(q(3) - phi_des) + kv_hip*qdot(3);
        
%     case 4 %Unloading
%         u(1) = -kp_L0*(q(5) - L_flight) - kv_L0*qdot(5);
%         u(2) = 0; %Zero Hip Torque
    case 3 %Flight
        u(1) = -kp_L0*(q(5) - 0.4) - kv_L0*qdot(5);
        u(2) = -kp_swing*((q(4)+q(3)) - footTDAngle) - kv_swing*(qdot(4)+qdot(3)); %Swing leg forward
        disp(u(2));
        
end
 
    if sum(imag(q)) + sum(imag(qdot)) ~= 0
        keyboard
    end
    
end

