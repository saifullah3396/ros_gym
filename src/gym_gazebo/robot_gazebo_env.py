import rospy
import gym
from gym.utils import seeding
from gazebo_handler import GazeboHandler
from mavros_gym_msgs.msg import RLExperimentInfo
from robot_sim_env import RobotSimEnv

class RobotGazeboEnv(RobotSimEnv):

    def __init__(self, robot_name_space, update_physics_params_at_start=True):
        super(RobotGazeboEnv, self).__init__(robot_name_space, GazeboHandler(update_physics_params_at_start))
