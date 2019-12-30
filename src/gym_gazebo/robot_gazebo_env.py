import rospy
import gym
import numpy as np
from gym.utils import seeding
from gazebo_handler import GazeboHandler
from mavros_gym_msgs.msg import RLExperimentInfo
from robot_sim_env import RobotSimEnv, WorldState

class RobotGazeboEnv(RobotSimEnv, WorldState):
    def __init__(self, robot_name_space, update_physics_params_at_start=True):
        super(RobotGazeboEnv, self).__init__(robot_name_space, GazeboHandler(update_physics_params_at_start))
        
    @property
    def front_camera(self):
        raise NotImplementedError
    
    @property
    def collision_check(self):
        raise NotImplementedError
