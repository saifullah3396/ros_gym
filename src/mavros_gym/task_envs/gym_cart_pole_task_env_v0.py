#!/usr/bin/env python3
"""
Defines the GymCartPoleTaskEnv class.
"""

import math
import numpy as np
from gym import logger
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
        assert self.action_space.contains(action), \
            "%r (%s) invalid" % (action, type(action))
        state = self.state
        x, x_dot, theta, theta_dot = state
        force = self.force_mag if action == 1 else -self.force_mag
        costheta = math.cos(theta)
        sintheta = math.sin(theta)
        temp = \
            (
                force + self.polemass_length *
                theta_dot * theta_dot * sintheta
            ) / self.total_mass
        thetaacc = \
            (self.gravity * sintheta - costheta * temp) / (
                self.length * (
                    4.0/3.0 - self.masspole * costheta * costheta /
                    self.total_mass
                ))
        xacc = \
            temp - self.polemass_length * thetaacc * costheta / \
            self.total_mass
        if self.kinematics_integrator == 'euler':
            x = x + self.tau * x_dot
            x_dot = x_dot + self.tau * xacc
            theta = theta + self.tau * theta_dot
            theta_dot = theta_dot + self.tau * thetaacc
        else:  # semi-implicit euler
            x_dot = x_dot + self.tau * xacc
            x = x + self.tau * x_dot
            theta_dot = theta_dot + self.tau * thetaacc
            theta = theta + self.tau * theta_dot
        self.state = (x, x_dot, theta, theta_dot)
        done = \
            x < -self.x_threshold \
            or x > self.x_threshold \
            or theta < -self.theta_threshold_radians \
            or theta > self.theta_threshold_radians
        done = bool(done)

        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this environment has"
                    " already returned done = True. You should always call"
                    " 'reset()' once you receive 'done = True' -- any further "
                    "steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        return {"robot_state": self.state}, reward, done, {}

    def reset(self):
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
        self.steps_beyond_done = None
        return {"robot_state": self.state}
