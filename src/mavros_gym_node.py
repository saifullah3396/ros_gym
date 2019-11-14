#!/usr/bin/env python

import numpy
import time

# ROS packages required
import rospy
import rospkg
import roslaunch
import subprocess

from mavros_gym import MavrosGym

if __name__ == '__main__':
    rospy.init_node('mavros_gym_training_node', anonymous=True, log_level=rospy.INFO)
    mavros_gym = MavrosGym()
    mavros_gym.setup()
    mavros_gym.start_training()
    rospy.spin()