# MuJoCo_Hopper
This is a MuJoCo based one legged hopper with foot simulation. It is designed to allow controller development and high level simulation control in Matlab through a compiled C library.  

For other branches, I was using SimulateHopper.m as the main file. However, for this trajectory optimization branch, the main file is optimization_stand_up.m.

One of the main functions is get_qdd in SLIP(), which is used to calculate qdd to enforce constraints.

I do not plan to use the Raibert style hopper, even though functionality related to it are still present here.

The C files that get compiled into library are slip.c and slip.h.

Add your MuJoCo library files into the mjpro150 folder, with your activation key.
