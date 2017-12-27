clear
close all
addpath('src');

slipObj = SLIP(1);

nStep = 12000;
q = zeros(6,nStep);

for i = 1:nStep
   slipObj.set_motor_command([0,0]); 
   slipObj.step();
   state = slipObj.get_state();
   
   if mod(i,8) == 0
        slipObj.draw();
   end
   
   
   q(:,i) = state.q;
end

slipObj.close();
