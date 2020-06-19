# pylint: disable=missing-module-docstring
import sys
import rospy
from gym_cart_pole_task_env_v0 import GymCartPoleTaskEnv


rospy.init_node("gym_cart_pole_task_env")

env = GymCartPoleTaskEnv()
rospy.spin()