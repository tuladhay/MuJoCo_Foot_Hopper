# MuJoCo_Hopper
This is a MuJoCo based one legged hopper with point foot simulation. It is designed to allow controller development and high level simulation control in Matlab through a compiled C library.  

The main Matlab file is SimulateHopper.m. This can be used for plotting and also controlling.
The C files that get compiled into library are slip.c and slip.h.
Currently, a Raibert style controller is implemented in C directly, and is being called from Matlab. The raibert controller in Matlab is not functional.
Add your MuJoCo library files into the mjpro150 folder, with your activation key.

Notes:
nQ = 5 [rootx, rootz, leg_tau, leg_motor, toe_spring]
nU = 2 [leg_tau, leg_motor]
nC = 1 [toe]

Rest leg length = 1.025m, (1.2 -0.25(toe) + 0.15/2). Might need to recheck.

The controller needs to be called from SimulateHopper by calling slipObj.controller(). This calls the raibert controller inside slip.c. I get confused because in branch-python_rl_integration, the step function itself calls the controller.
