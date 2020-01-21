#!/usr/bin/env python3
"""
Defines the UAVBaseTaskEnv class.
"""

import numpy as np
from gym.spaces import Box, Dict
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped


class UAVBaseTaskEnv():
    """
    Initializes the base parameters of the environment common to all task
    environments related to UAVs.
    """
    # pylint: disable=attribute-defined-outside-init
    def __init__(self):
        self._setup()

    def _setup(self):
        self._setup_workspace()
        self._setup_action_space()
        self._setup_init_action_params()
        self._setup_desired_pose()
        self._setup_reward_params()
        self.desired_pose_epsilon = \
            rospy.get_param("/mavros_gym/desired_point_epsilon")
        self.geo_distance = \
            rospy.get_param("/mavros_gym/geodesic_distance")
        self.vel_msg = TwistStamped()
        self.rate = rospy.Rate(1000.0)
        self.use_pose_estimator = \
            rospy.get_param("/mavros_gym/use_pose_estimator")
        self.min_height = rospy.get_param("mavros_gym/min_height")

    def _setup_workspace(self):
        """
        Sets up the workspace of the environment.
        """
        self.work_space_x_max = rospy.get_param("/mavros_gym/work_space/x_max")
        self.work_space_x_min = rospy.get_param("/mavros_gym/work_space/x_min")
        self.work_space_y_max = rospy.get_param("/mavros_gym/work_space/y_max")
        self.work_space_y_min = rospy.get_param("/mavros_gym/work_space/y_min")
        self.work_space_z_max = rospy.get_param("/mavros_gym/work_space/z_max")
        self.work_space_z_min = rospy.get_param("/mavros_gym/work_space/z_min")

        # min/max reward
        self.reward_range = (-np.inf, np.inf)

        # maximum quaternion values
        self.max_qw = rospy.get_param("/mavros_gym/max_orientation_w")
        self.max_qx = rospy.get_param("/mavros_gym/max_orientation_x")
        self.max_qy = rospy.get_param("/mavros_gym/max_orientation_y")
        self.max_qz = rospy.get_param("/mavros_gym/max_orientation_z")

        # maximum velocity values
        self.max_vel_lin_x = \
            rospy.get_param("/mavros_gym/max_velocity_vector/linear_x")
        self.max_vel_lin_y = \
            rospy.get_param("/mavros_gym/max_velocity_vector/linear_y")
        self.max_vel_lin_z = \
            rospy.get_param("/mavros_gym/max_velocity_vector/linear_z")
        self.max_vel_ang_x = \
            rospy.get_param("/mavros_gym/max_velocity_vector/angular_x")
        self.max_vel_ang_y = \
            rospy.get_param("/mavros_gym/max_velocity_vector/angular_y")
        self.max_vel_ang_z = \
            rospy.get_param("/mavros_gym/max_velocity_vector/angular_z")

        # front camera resolution
        self.front_cam_h = rospy.get_param("/mavros_gym/front_cam_res/height")
        self.front_cam_w = rospy.get_param("/mavros_gym/front_cam_res/width")

        pos_obs_low = \
            np.array([
                self.work_space_x_max,
                self.work_space_y_max,
                self.work_space_z_max,
                -self.max_qw,
                -self.max_qx,
                -self.max_qy,
                -self.max_qz])

        vel_obs_low = \
            np.array([
                -self.max_vel_lin_x,
                -self.max_vel_lin_y,
                -self.max_vel_lin_z,
                -self.max_vel_ang_x,
                -self.max_vel_ang_y,
                -self.max_vel_ang_z])

        pos_obs_high = \
            np.array([
                self.work_space_x_max,
                self.work_space_y_max,
                self.work_space_z_max,
                self.max_qw,
                self.max_qx,
                self.max_qy,
                self.max_qz])

        vel_obs_high = \
            np.array([
                self.max_vel_lin_x,
                self.max_vel_lin_y,
                self.max_vel_lin_z,
                self.max_vel_ang_x,
                self.max_vel_ang_y,
                self.max_vel_ang_z])

        self.pos_obs_space = \
            Box(
                pos_obs_low,
                pos_obs_high,
                dtype=np.float32)
        self.vel_obs_space = \
            Box(
                vel_obs_low,
                vel_obs_high,
                dtype=np.float32)

        self.front_cam_obs_space = \
            Box(
                low=0,
                high=255,
                shape=(self.front_cam_h, self.front_cam_w, 4),
                dtype=np.uint8)

        self.observation_space = \
            Dict({
                'position': self.pos_obs_space,
                'velocity': self.vel_obs_space,
                'front_cam': self.front_cam_obs_space})

    def _setup_action_space(self):
        """
        Sets up the robot action space.
        """
        # generate a continuous action space
        hv_range = rospy.get_param('/mavros_gym/lxy_vel_range')
        vv_range = rospy.get_param('/mavros_gym/lz_vel_range')
        rv_range = rospy.get_param('/mavros_gym/rot_vel_range')

        self.action_low = np.array([-1*hv_range, -1*hv_range, -1*vv_range,
                                    -1*rv_range])
        self.action_high = np.array([hv_range, hv_range, vv_range, rv_range])
        self.action_space = \
            Box(low=self.action_low, high=self.action_high, dtype=np.float32)

    def _setup_init_action_params(self):
        """
        Set the initial robot velocity.
        """
        self.init_velocity = TwistStamped()
        self.init_velocity.twist.linear.x = \
            rospy.get_param('/mavros_gym/init_speed_vector/linear_x')
        self.init_velocity.twist.linear.y = \
            rospy.get_param('/mavros_gym/init_speed_vector/linear_y')
        self.init_velocity.twist.linear.z = \
            rospy.get_param('/mavros_gym/init_speed_vector/linear_z')
        self.init_velocity.twist.angular.x = \
            rospy.get_param('/mavros_gym/init_speed_vector/angular_x')
        self.init_velocity.twist.angular.y = \
            rospy.get_param('/mavros_gym/init_speed_vector/angular_y')
        self.init_velocity.twist.angular.z = \
            rospy.get_param('/mavros_gym/init_speed_vector/angular_z')

    def _setup_desired_pose(self):
        """
        Set the desired final pose of the robot.
        """
        self.desired_pose = PoseStamped()
        self.desired_pose.pose.position.x = \
            rospy.get_param("/mavros_gym/desired_position/x")
        self.desired_pose.pose.position.y = \
            rospy.get_param("/mavros_gym/desired_position/y")
        self.desired_pose.pose.position.z = \
            rospy.get_param("/mavros_gym/desired_position/z")
        self.desired_pose.pose.orientation.w = \
            rospy.get_param("/mavros_gym/desired_orientation/w")
        self.desired_pose.pose.orientation.x = \
            rospy.get_param("/mavros_gym/desired_orientation/x")
        self.desired_pose.pose.orientation.y = \
            rospy.get_param("/mavros_gym/desired_orientation/y")
        self.desired_pose.pose.orientation.z = \
            rospy.get_param("/mavros_gym/desired_orientation/z")

    def _setup_reward_params(self):
        """
        Sets the reward parameters.
        """
        self.closer_to_point_reward = \
            rospy.get_param("/mavros_gym/closer_to_point_reward")
        self.not_ending_point_reward = \
            rospy.get_param("/mavros_gym/not_ending_point_reward")
        self.end_episode_points = \
            rospy.get_param("/mavros_gym/end_episode_points")
        self.collision_penalty = \
            rospy.get_param("/mavros_gym/collision_penalty")
