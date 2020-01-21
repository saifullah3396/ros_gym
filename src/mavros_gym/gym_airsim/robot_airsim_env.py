#!/usr/bin/env python3
"""
Defines the RobotAirSimEnv class.
"""

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from robot_sim_env import RobotSimEnv, WorldState
from .airsim_handler import AirsimHandler


class RobotAirSimEnv(RobotSimEnv, WorldState):
    """
    The base class for any kind of robot that uses the airsim client.
    All functionality that is provided by airsim and is common between any
    type of robot is defined here.
    """
    def __init__(self):
        super(RobotAirSimEnv, self).__init__(AirsimHandler())

    @staticmethod
    def airsim_to_ros_pose(airsim_position, airsim_orientation):
        """
        Converts airsim pose to ros pose message.

        Parameters
        ----------
        airsim_position:  Airsim Position Type
        airsim_orientation: Airsim Orientation Type

        Returns
        -------
        ros_pose: PoseStamped
        """
        ros_pose = PoseStamped()
        ros_pose.pose.position.x = airsim_position.x_val
        ros_pose.pose.position.y = airsim_position.y_val
        ros_pose.pose.position.z = airsim_position.z_val
        ros_pose.pose.orientation.w = airsim_orientation.w_val
        ros_pose.pose.orientation.x = airsim_orientation.x_val
        ros_pose.pose.orientation.y = airsim_orientation.y_val
        ros_pose.pose.orientation.z = airsim_orientation.z_val
        return ros_pose

    @staticmethod
    def airsim_to_ros_twist(airsim_lin_vel, airsim_ang_vel):
        """
        Converts airsim twist to ros twist message.

        Parameters
        ----------
        airsim_position:  Airsim Linear Velocity Type
        airsim_orientation: Airsim Angular Velocity Type

        Returns
        -------
        ros_twist: TwistStamped
        """
        ros_twist = TwistStamped()
        ros_twist.twist.linear.x = airsim_lin_vel.x_val
        ros_twist.twist.linear.y = airsim_lin_vel.y_val
        ros_twist.twist.linear.z = airsim_lin_vel.z_val
        ros_twist.twist.angular.x = airsim_ang_vel.x_val
        ros_twist.twist.angular.y = airsim_ang_vel.y_val
        ros_twist.twist.angular.z = airsim_ang_vel.z_val
        return ros_twist

    @staticmethod
    def airsim_image_to_numpy(airsim_img):
        """
        Converts airsim image to numpy image.

        Parameters
        ----------
        airsim_img:  Airsim Image Type

        Returns
        -------
        img_rgba: np.array
            An RGBA image as numpy array
        """
        img1d = \
            np.fromstring(
                airsim_img.image_data_uint8, dtype=np.uint8)
        img_rgba = \
            img1d.reshape(
                airsim_img.height, airsim_img.width, 4)
        img_rgba = np.flipud(img_rgba)
        return img_rgba

    @staticmethod
    def airsim_depth_image_to_numpy(airsim_img):
        """
        Converts airsim image to numpy image.

        Parameters
        ----------
        airsim_img:  Airsim Image Type

        Returns
        -------
        img_depth: np.array
            Image depth as numpy array
        """
        img_depth = \
            np.array(airsim_img.image_data_float, dtype=np.float32)
        img_depth = img_depth.reshape(airsim_img.height, airsim_img.width)
        return img_depth

    def camera(self, camera_index):
        """
        Returns the front camera image.
        """
        return self.airsim_image_to_numpy(
            self.sim_handler.client_camera(camera_index))

    def camera_depth(self, camera_index):
        """
        Returns the front camera image depth.
        """
        return self.airsim_depth_image_to_numpy(
            self.sim_handler.client_camera_depth(camera_index))

    @property
    def collision_check(self):
        """
        Returns whether collision is true or not.
        """
        return self.sim_handler.client_collision_check
