gain_footDisp = 0.09;
des_velocity = 2;
L_flight = 0.45;

qd = -1.5;
NP=0.5*(qd*0.1)
xf_Point = -NP + gain_footDisp*(-qd - des_velocity)
des_td_angle = asin(xf_Point/L_flight)
disp(rad2deg(des_td_angle))