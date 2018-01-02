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
hopper_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
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
        self.s = hopper_init('.')
        self.v = hopper_vis()
        self.wait = False

        self.nA = 1		# number of controllable parameters for rl
        self.nS = 8		# number of inputs to feed to rl
        self.nSO = 3

        self.action = np.zeros(self.nA, dtype=np.double)
        self.state = np.zeros(self.nS, dtype=np.double)

        self.stats_keys = {'VelocityTracking', 'Torque', 'Complete'}
        self.state_keys = {'x','z','xd','zd','tau_motor_pos','foot_motor_pos','foot_heel_pos','foot_toe_pos','dynamic_state'}
        # height tracking might be redundant if minimum torque is also an objective

    def reset(self):
        # Resets the state of the environment, returning an initial observation.
        #
        # Outputs
        # -------
        # observation : the initial observation of the space. (Initial reward is assumed to be 0.)
        # self.state = np.array([0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # need to add dynamic state
        # the cpos are set to 0, but this is not correct, so,
        # Make a c function that resets these states (by passing a pointer?)

        hopper_reset(self.s)
        self.action = 0.0
        hopper_step(self.s, action.ctypes.data_as(c_double_p))      # just to get the first state
        hopper_obs(self.s, self.state.ctypes.data_as(c_double_p))

        temp = np.copy(self.state)
        return temp

    def step(self, action):
        # INPUT : Action provided by the environment
        #
        # OUTPUTS :Observation, reward, done, info
        # observation: agent's observation of the current environment
        # reward [float] : amount of reward due to the previous action
        # done : boolean, indicating whether the episode has ended
        # info : a dictionary containing other diagnostic information from the previous action

        self.stats = np.zeros(self.nSO, dtype=np.double) #reset stats

        # Execute one time-step using action
        hopper_step(self.s, action.ctypes.data_as(c_double_p))

        # Observations
        hopper_obs(self.s, self.state.ctypes.data_as(c_double_p))  # this is supposed to update self.state
        
        # Set the DONE flag
        done = False
        if (self.state[1] < 0.4):
            done = True

        # Rewards
        weight_vel_tracking = 1e-2
        weight_torque = 1e-4            # torque for foot motor
        target_vel = 2                  # m/s

        r = 0.0
        if not done:
            r += 1                      # reward for staying alive

        r -= weight_vel_tracking * np.absolute(target_vel - self.state[2])
        r -= weight_torque * np.absolute(action)       # action is the foot torque

        # TODO:
        # In state_t, also add the apex height so that it can be used as tracking in objective
        # weight_hei_tracking = 1e-4
        # however, height tracking can be redundant if min torque is also an objective

        rendered = hopper_draw(self.v, self.s, self.wait)

        print r
        #return Step(observation = self.state, reward = r, done = done)
        

    @cached_property
    def observation_space(self):
        high = np.array([np.inf, 5.0, 5.0, 20.0, 0.25, 0.78, 5.0, 5.0, 3])
        low = np.array([-np.inf, 0.0, -5.0, -20.0, -0.25, -0.78, -5.0, -5.0, 1])
        return Box(low, high)

    @cached_property
    def action_space(self):
        high = np.array([500.0])    # foot motor max torque
        low = np.array([-500.0])    # foot motor min torque
        return Box(low, high)


hopper = footHopperEnv()
print(hopper.action)
for _ in range(1000):
    hopper.step(hopper.action)
