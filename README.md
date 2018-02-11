# MuJoCo_Hopper
This is a MuJoCo based one legged hopper with foot simulation. It is designed to allow controller development and high level simulation control in Matlab/Python through a compiled C library.  

Currently, a Raibert style controller is implemented in C directly, and is being called from Matlab. The raibert controller in Matlab is not functional.
Add your MuJoCo library files into the mjpro150 folder, with your activation key.



Make sure rllab and gym are in your path, and added in your project structure.
For eg, I had them inside my pycharm-community folder.

I created a pycharm project with "MuJoCo_Foot_Hopper", "gym" "rllab" and "anaconda" were in my content root (in project structure).

rllab/vendor was marked in "source folders". However I dont know if it needs to be marked as such.

"make" the c files.

Use python 2.7 interpreter.

Install whatever other libraries it does not find automatically.
