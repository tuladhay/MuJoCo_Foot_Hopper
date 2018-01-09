clear raibertController
clear all
close all
addpath('src');
addpath('controllers');

slipObj = SLIP(1);

nStep = 3500;
q = zeros(slipObj.nQ, nStep);        % to store output
qdot = zeros(slipObj.nQ,nStep);
pos_heel = zeros(1,nStep);
pos_toe = zeros(1,nStep);
pos_slider = zeros(1,nStep);
tau_motor = zeros(3,nStep);
des_td_arr = zeros(1,nStep);
apex_velocity = zeros(1,nStep);

initState = slipObj.blank_state();   % all states set to 0, dynamic_state = 3
initState.q = [0,1.2,0,0.45,0,0];    %[x, y, leg_tau, leg_motor, leg_spring, foot_joint]
% TODO: Implement switch case for starting with some positive velocity, as
% done in point_foot
slipObj.set_state(initState);

for i = 1:nStep
   %slipObj.set_motor_command([u(2) u(1)]); 
   slipObj.step();
   state = slipObj.get_state();
   slipObj.controller();

   dyn = state.dynamic_state;
   des_td_angle = state.des_td_angle;
   des_td_arr(:,i) = des_td_angle;
   
   if dyn ==1
        disp('Compression');
   elseif dyn==2
        disp('Thrust');
   elseif dyn==3
        disp('Flight');
   end
   
   %disp(state.cpos)
   %disp(state.u)
   %disp(state.stance_time)
   
   pos_heel(:,i) = state.cpos(1);
   pos_toe(:,i) = state.cpos(2);
   pos_slider(:,i) = state.q(4);
   tau_motor(:,i) = state.u;
   apex_velocity(:,i) = state.apex_velocity;
   q(:,i) = state.q;
   qdot(:,i) = state.qd;
   
   if mod(i,1) == 0
       slipObj.draw();
   end
   
end
% figure(1)
% plot(pos_heel);
% figure(2)
% plot(pos_toe);

figure(3)
plot(pos_slider);
filename = 'slide motor position';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(4)
plot(tau_motor(1,:));
filename = 'leg tau torque';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(5)
plot(tau_motor(2,:));
filename = 'leg slide motor torque';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(6)
plot(des_td_arr);
filename = 'desired touchdown angle';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(7)
plot(qdot(1,:));
filename = 'X velocity';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(8)
plot(q(1,:));
filename = 'X position';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(9)
plot(q(3,:));
filename = 'leg tau motor position';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(10)
plot(apex_velocity(1,:));
filename = 'Apex velocity';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

figure(11)
plot(q(2,:));
filename = 'Z position';
title(filename);
saveas(gcf, strcat(pwd, strcat('/Plot/', filename)));

slipObj.close();
