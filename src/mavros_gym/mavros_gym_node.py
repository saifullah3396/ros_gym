#!/usr/bin/env python3
""" Initializes the mavros_gym ros node for training. """

import rospy
from .mavros_gym import MavrosGym


if __name__ == '__main__':
    rospy.init_node(
        'mavros_gym_node', anonymous=True, log_level=rospy.INFO)
    # pylint: disable=invalid-name
    mavros_gym_node = MavrosGym()
    mavros_gym_node.setup()
    mavros_gym_node.start_training()
    rospy.spin()
