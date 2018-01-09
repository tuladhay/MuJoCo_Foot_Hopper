clear raibertController
clear all
close all
addpath('src');
addpath('controllers');

slipObj = SLIP(1);

nStep = 3500;
q = zeros(slipObj.nQ,nStep);        % to store output
qdot = zeros(slipObj.nQ,nStep);
pos_heel = zeros(1,nStep);
pos_toe = zeros(1,nStep);
pos_slider = zeros(1,nStep);
tau_motor = zeros(slipObj.nU,nStep);
des_td_arr = zeros(1,nStep);
apex_velocity = zeros(1,nStep);

% initState = slipObj.blank_state();
% initState.q = [0,1.2,0,0,0.45,0,0]; %0.4 for 5th position    %[x, y, phi(rot), leg_tau, leg_motor, leg_spring]
% slipObj.set_state(initState);
% slipObj.set_dynamic_state();

for i = 1:nStep
   slipObj.step(1000);      % pass value for ankle torque

% see the slipObj.slip function
   state = slipObj.get_state();     % from mjdata
   
   %dyn = state.dynamic_state;
   %des_td_angle = slipObj.get_des_td_angle();
   %des_td_arr(:,i) = des_td_angle;
   
%    if dyn ==1
%         disp('Compression');
%    elseif dyn==2
%         disp('Thrust');
%    elseif dyn==3
%         disp('Flight');
%    end
   
   %disp(state.cpos)
   %disp(state.u)
   %disp(state.stance_time)
   
%    pos_heel(:,i) = state.cpos(1);
%    pos_toe(:,i) = state.cpos(2);
%    pos_slider(:,i) = state.q(5);
%    %tau_motor(:,i) = state.u;
%    apex_velocity(:,i) = state.apex_velocity;
%    q(:,i) = state.q;
%    qdot(:,i) = state.qd;
   
   if mod(i,5) == 0
       slipObj.draw();
   end
   
end
% figure(1)
% plot(pos_heel);
% figure(2)
% plot(pos_toe);

figure(3)
plot(pos_slider);
title('leg slide motor position');

figure(4)
plot(tau_motor(1,:));
title('leg rot motor torque');

figure(5)
plot(tau_motor(2,:));
title('leg slide motor torque');

figure(6)
plot(des_td_arr);
title('desired touchdown angle');

figure(7)
plot(qdot(1,:));
title('X velocity');

figure(8)
plot(q(1,:));
title('X position');

figure(9)
plot(q(4,:));
title('leg rotational motor position');

figure(10)
plot(apex_velocity(1,:));
title('Apex Velocity');

slipObj.close();
