#!/usr/bin/env python3
""" Initializes the ros_gym ros node for training. """

import rospy
from ros_gym import MavrosGym


if __name__ == '__main__':
    rospy.init_node(
        'ros_gym_node', anonymous=True, log_level=rospy.INFO)
    # pylint: disable=invalid-name
    ros_gym_node = MavrosGym()
    ros_gym_node.setup()
    ros_gym_node.start_training()
    rospy.spin()
