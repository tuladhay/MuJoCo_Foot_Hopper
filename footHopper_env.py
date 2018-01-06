import ctypes
from ctypes import cdll
import numpy as np
from rllab.envs.base import Env
from rllab.envs.base import Step
from rllab.spaces import Box
from cached_property import cached_property

import time

# This file is creating an environment for the SLIP hopper with foot.
# This wraps around a C library compiled from slip.c
# This environment will be used with rllab to test RL algorithms on the SLIP foot hopper.


# Loading the compiled C library.
lib = cdll.LoadLibrary('./libslip.so')
c_double_p = ctypes.POINTER(ctypes.c_double)

# To call Init function
hopper_init = lib.init
hopper_init.argtypes = [ctypes.c_char_p]
hopper_init.restype = ctypes.c_void_p

# To call Step function
hopper_step = lib.step          # (slip_t* s, double action)
hopper_step.argtypes = [ctypes.c_void_p, ctypes.c_double]
hopper_step.restype = None

# To call get_observation function
hopper_obs = lib.get_observation            # (slip_t* s, double* state_vec)
hopper_obs.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
hopper_obs.restype = None

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


class footHopperEnv(Env):

    def __init__(self):
        self.s = hopper_init('.')           # returns mjdata struct
        #self.v = hopper_vis()               # returns vis_init struct
        self.wait = False                   # visualization wait for user argument

        self.nA = 1		                    # number of controllable parameters for rl
        self.nS = 8		                    # number of inputs to feed to rl

        self.action = np.zeros(self.nA, dtype=np.double)        # inputs to step function in c
        self.state = np.zeros(self.nS, dtype=np.double)         # inputs to feed rl observation

        self.state_keys = {'x','z','xd','zd','tau_motor_pos','foot_motor_pos','foot_heel_pos','foot_toe_pos'}
        # height tracking might be redundant if minimum torque is also an objective

        # Debugging variables
        self.reward = 0.0
        self.done = False
        self.counter = 0
        self.action_prev = self.action      # so that they are the same type

    def reset(self):
        # Resets the state of the environment, returning an initial observation.
        #
        # Outputs
        # -------
        # observation : the initial observation of the space. (Initial reward is assumed to be 0.)

        hopper_reset(self.s)                                        # calls set_initial_state()
        self.action = np.zeros(self.nA, dtype=np.double)
        hopper_step(self.s, ctypes.c_double(0.0))                   # just to get the first state
        hopper_obs(self.s, self.state.ctypes.data_as(c_double_p))   # send state data in array

        temp = np.copy(self.state)
        return temp

    def step(self, action):
        # INPUT : Action provided by the environment
        #
        # OUTPUTS :Observation, reward, done, info
        # observation: agent's observation of the current environment
        # reward [float] : amount of reward due to the previous action
        # done : boolean, indicating whether the episode has ended

        # Execute one or many time-step/s using action
        for _ in range(10):
            hopper_step(self.s, ctypes.c_double(action))

        # Observations
        hopper_obs(self.s, self.state.ctypes.data_as(c_double_p))  # this is supposed to update self.state
        
        # Set the DONE flag
        done = False
        if (self.state[1] < 0.85):
            done = True
        self.done = done                # to check from outside

        # Rewards
        weight_vel_tracking = 1e-3
        weight_torque = 0.05            # torque for foot motor
        target_vel = 1.65                # m/s
        weight_jerkiness = 1e-4
        weight_neg_direction = 100
        weight_normal_foot_pos = 1

        r = 0.0                         # Even though set to 0 here, rllab will add the rewards
        if not done:
            r += 1                      # reward for staying alive

        r -= weight_vel_tracking * np.square( np.absolute(target_vel - self.state[2]) )       # may need to square the error
        r -= weight_torque * np.absolute(action)       # action is the foot torque
        # r -= weight_jerkiness * np.absolute(self.action_prev - action)      # for smooth ankle torque profiles
        # r -= weight_neg_direction * np.absolute(self.state[5])
        if self.state[2] > 0:
            r += 1
        r -= 1.23 - self.state[1]
        # if self.state[2] < 0:
        #     r -= weight_neg_direction * np.absolute(self.state[2])   # if x vel is negative

        self.action_prev = action

        # TODO:
        # In state_t, also add the apex height so that it can be used as tracking in objective
        # However, height tracking can be redundant if min torque is also an objective

        return Step(observation=self.state, reward=r[0], done=done)

    def render(self):
        if self.counter == 0:
            self.v = hopper_vis()
            self.counter = 1

        hopper_draw(self.v, self.s, self.wait)

    @cached_property
    def observation_space(self):
        high = np.array([np.inf, 2.0, 5.0, 20.0, 0.5, 0.8, 1.0, 1.0])
        low = np.array([-np.inf, 0.0, -5.0, -20.0, -0.5, -0.8, -0.1, -0.1])         # need to change ground compliance
        return Box(low, high)

    @cached_property
    def action_space(self):

        high = np.array([50.0])    # foot motor max torque
        low = np.array([-10.0])    # foot motor min torque
        return Box(low, high)


# Unit testing (COMMENT OUT WHEN RUNNING RLLAB)
#
# hopper = footHopperEnv()
# print(hopper.action)
# for _ in range(2000):
#     print('Action = ' + str(hopper.action) + ', Reward = ' + str(hopper.reward))
#     result = hopper.step(hopper.action)
#     hopper.render()
#     if _ == 500:
#         hopper.reset()
#         print("Resetting")
#
#     if _ == 1500:
#         hopper.reset()
#         print("Resetting")
