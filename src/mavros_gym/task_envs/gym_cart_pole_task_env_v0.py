#!/usr/bin/env python3
"""
Defines the GymCartPoleTaskEnv class.
"""

import numpy as np
from gym.envs.classic_control import CartPoleEnv
from gym.spaces import Box, Dict


class GymCartPoleTaskEnv(CartPoleEnv):
    """
    This class wraps the gym cartpole environment.
    """
    def __init__(self):
        super(GymCartPoleTaskEnv, self).__init__()
        # Angle limit set to 2 * theta_threshold_radians so failing observation
        # is still within bounds
        high = np.array([
            self.x_threshold * 2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians * 2,
            np.finfo(np.float32).max])
        self.observation_space = \
            Dict({"robot_state": Box(-high, high, dtype=np.float32)})

    def step(self, action):
        state, reward, done, _ = super(GymCartPoleTaskEnv, self).step(action)
        return {"robot_state": state}, reward, done, {}

    def reset(self):
        state = super(GymCartPoleTaskEnv, self).reset()
        return {"robot_state": state}
