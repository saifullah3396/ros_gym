#!/usr/bin/env python3
"""
Defines the GymCartPoleTaskEnv class.
"""

import numpy as np
from gym.envs.classic_control import Continuous_MountainCarEnv
from gym.spaces import Box, Dict


class GymMCContinuousTaskEnv(Continuous_MountainCarEnv):
    """
    This class wraps the gym continuous mountain car environment.
    """
    def __init__(self):
        super(GymMCContinuousTaskEnv, self).__init__()
        # Angle limit set to 2 * theta_threshold_radians so failing observation
        # is still within bounds
        self.observation_space = \
            Dict({
                "robot_state": Box(
                    low=self.low_state,
                    high=self.high_state,
                    dtype=np.float32)})

    def step(self, action):
        state, reward, done, _ = \
            super(GymMCContinuousTaskEnv, self).step(action)
        return {"robot_state": state}, reward, done, {}

    def reset(self):
        state = super(GymMCContinuousTaskEnv, self).reset()
        return {"robot_state": state}
