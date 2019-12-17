#!/usr/bin/env python
import numpy as np
import rospy
from gym import spaces
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion

class UAVBaseTaskEnv():
    def __init__(self):    
        """
        Initializes the base parameters of the environment from yaml configuration
        """
        self._setup()

    def _setup(self):
        self._setup_workspace()
        self._setup_action_space()
        self._setup_init_velocity()
        self._setup_desired_pose()
        self._setup_rewards
        self.desired_pose_epsilon = rospy.get_param("/mavros_gym/desired_point_epsilon")
        self.geo_distance = rospy.get_param("/mavros_gym/geodesic_distance")
        self.cumulated_steps = 0.0
        self.vel_msg = TwistStamped()
        self.rate = rospy.Rate(1000.0)
        self.use_pose_estimator = rospy.get_param("/mavros_gym/use_pose_estimator")

    def _setup_workspace(self):
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

        high = \
            np.array([
                self.work_space_x_max, 
                self.work_space_y_max, 
                self.work_space_z_max,
                self.max_qw, 
                self.max_qx, 
                self.max_qy, 
                self.max_qz])

        low = \
            np.array([ \
                self.work_space_x_min,
                self.work_space_y_min,
                self.work_space_z_min,
                -1*self.max_qw,
                -1*self.max_qx,
                -1*self.max_qy,
                -1*self.max_qz])

        self.observation_space = spaces.Box(low, high)

    def _setup_action_space(self):
        # Generate a continuous action space
        hv_range = rospy.get_param('/mavros_gym/lxy_vel_range')
        vv_range = rospy.get_param('/mavros_gym/lz_vel_range')
        rv_range = rospy.get_param('/mavros_gym/rot_vel_range')
        
        self.action_space = \
            spaces.Box(
                low=np.array([-hv_range, -hv_range, -vv_range, -rv_range]), \
                high=np.array([hv_range, hv_range, vv_range, rv_range]))

    def _setup_init_velocity(self):
        self._init_velocity = TwistStamped()
        self._init_velocity.twist.linear.x = \
            rospy.get_param('/mavros_gym/init_speed_vector/linear_x')
        self._init_velocity.twist.linear.y = \
            rospy.get_param('/mavros_gym/init_speed_vector/linear_y')
        self._init_velocity.twist.linear.z = \
            rospy.get_param('/mavros_gym/init_speed_vector/linear_z')
        self._init_velocity.twist.angular.x = \
            rospy.get_param('/mavros_gym/init_speed_vector/angular_x')
        self._init_velocity.twist.angular.y = \
            rospy.get_param('/mavros_gym/init_speed_vector/angular_y')
        self._init_velocity.twist.angular.z = \
            rospy.get_param('/mavros_gym/init_speed_vector/angular_z')

    def _setup_desired_pose(self):
        self.desired_pose = PoseStamped()
        self.desired_pose.pose.position.x = rospy.get_param("/mavros_gym/desired_position/x")
        self.desired_pose.pose.position.y = rospy.get_param("/mavros_gym/desired_position/y")
        self.desired_pose.pose.position.z = rospy.get_param("/mavros_gym/desired_position/z")
        self.desired_pose.pose.orientation.w = rospy.get_param("/mavros_gym/desired_orientation/w")
        self.desired_pose.pose.orientation.x = rospy.get_param("/mavros_gym/desired_orientation/x")
        self.desired_pose.pose.orientation.y = rospy.get_param("/mavros_gym/desired_orientation/y")
        self.desired_pose.pose.orientation.z = rospy.get_param("/mavros_gym/desired_orientation/z")

    def _setup_rewards(self):
        self.closer_to_point_reward = \
            rospy.get_param("/mavros_gym/closer_to_point_reward")
        self.not_ending_point_reward = \
            rospy.get_param("/mavros_gym/not_ending_point_reward")
        self.end_episode_points = \
            rospy.get_param("/mavros_gym/end_episode_points")