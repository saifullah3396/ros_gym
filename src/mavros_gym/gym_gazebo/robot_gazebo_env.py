#!/usr/bin/env python3
"""
Defines the RobotGazeboEnv class.
"""

from robot_sim_env import RobotSimEnv, WorldState
from .gazebo_handler import GazeboHandler


class RobotGazeboEnv(RobotSimEnv, WorldState):
    """
    The base class for all robots that use gazebo simulator for training.
    """
    def __init__(self, robot_name_space, update_physics_params_at_start=True):
        super(RobotGazeboEnv, self).__init__(
            robot_name_space, GazeboHandler(update_physics_params_at_start))

    @property
    def front_camera(self):
        """
        Derived from WorldState
        """
        raise NotImplementedError()

    @property
    def collision_check(self):
        """
        Derived from WorldState
        """
        raise NotImplementedError()
