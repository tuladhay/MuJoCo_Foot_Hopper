# MuJoCo_Hopper
This is a MuJoCo based one legged hopper with foot simulation. It is designed to allow controller development and high level simulation control in Matlab through a compiled C library.  

The main Matlab file is SimulateHopper.m. This can be used for plotting and also controlling.
The C files that get compiled into library are slip.c and slip.h.
Currently, a Raibert style controller is implemented in C directly, and is being called from Matlab. The raibert controller in Matlab is not functional.
Add your MuJoCo library files into the mjpro150 folder, with your activation key.
