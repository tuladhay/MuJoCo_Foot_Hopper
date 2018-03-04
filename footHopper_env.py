import ctypes
from ctypes import cdll
from state_struct import State
import numpy as np

import time

# This file is creating an environment for the SLIP hopper with foot.
# This wraps around a C library compiled from slip.c
# This environment will be used with rllab to test RL algorithms on the SLIP foot hopper.

# ********************************************************************************************************************
#                         Python variable/function definitions to wrap with C library
# ********************************************************************************************************************
# Loading the compiled C library. Make sure to 'make' your C files
lib = cdll.LoadLibrary('./libslip.so')
c_double_p = ctypes.POINTER(ctypes.c_double)

# To call Init function
hopper_init = lib.init
hopper_init.argtypes = [ctypes.c_char_p]
hopper_init.restype = ctypes.c_void_p

# To call Step function
hopper_step = lib.step
hopper_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(State)]
hopper_step.restype = None

# To call set_initial_state function
hopper_reset = lib.set_initial_state
hopper_reset.argtypes = [ctypes.c_void_p]
hopper_reset.restype = None

# To call visualization
hopper_vis = lib.vis_init
hopper_vis.argtypes = None
hopper_vis.restype = ctypes.c_void_p

# To call vis_draw
hopper_draw = lib.vis_draw
hopper_draw.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_bool]
hopper_draw.restype = ctypes.c_bool

# ********************************************************************************************************************
# ********************************************************************************************************************


class footHopperEnv():

    def __init__(self):
        self.s = hopper_init('.')           # returns mjdata struct
        # self.v = hopper_vis()               # returns vis_init struct
        self.wait = False                   # visualization wait for user argument
        self.state = State()
        self.counter = 0

    def step(self):
        # Execute one or many time-step/s using action
        for _ in range(10):
            hopper_step(self.s, self.state)
            # hopper_draw(self.v, self.s, self.wait)

    def render(self):
        if self.counter == 0:
            self.v = hopper_vis()
            self.counter = 1

        hopper_draw(self.v, self.s, self.wait)

    def controller(self):
        # POSITIONS
        pos_x = self.state.q[0]
        pos_z = self.state.q[1]
        pos_hip_motor = self.state.q[2]
        pos_leg_motor = self.state.q[3]
        pos_spring = self.state.q[4]
        heel_pos = self.state.cpos[0]
        toe_pos = self.state.cpos[1]

        # ACCELERATIONS
        acc_z = self.state.qdd[1]

        # VELOCITES
        # some_vel = self.state.q[id]
        vel_leg_motor = self.state.qd[3]

        # GAINS
        kp_leg = 850
        kd_leg = 100
        kp_hip = 500
        kd_hip = 15

        # PARAMETERS
        # The leg slide range is from 0.45 to 0.60, with 0.60 being fully extended
        L_flight = 0.5
        L_extension = 0.55

        if (heel_pos>0.0 and toe_pos>0.0):
            leg_torque = -kp_leg*(pos_leg_motor - L_flight) - kd_leg*(vel_leg_motor)
            self.state.u[1] = leg_torque
            a = 1

        if acc_z > 0.0 and (heel_pos<0.0 or toe_pos<0.0):
            self.state.u[1] = -kp_leg*(pos_leg_motor - L_extension) - kd_leg*(vel_leg_motor)


if __name__=="__main__":
    hopper = footHopperEnv()
    for _ in range(1000):
        hopper.render()
        hopper.step()
        hopper.controller()
    print("hi")
