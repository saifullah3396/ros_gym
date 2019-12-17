import rospy
import gym
from gym.utils import seeding
from airsim_handler import AirsimHandler
from mavros_gym_msgs.msg import RLExperimentInfo
from robot_sim_env import RobotSimEnv

class RobotAirSimEnv(RobotSimEnv):

    def __init__(self, robot_name_space):
        super(RobotAirSimEnv, self).__init__(robot_name_space, AirsimHandler())