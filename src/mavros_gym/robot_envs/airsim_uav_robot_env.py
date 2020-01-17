#!/usr/bin/env python3

import os
import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from gym_airsim import robot_airsim_env


class AirSimUAVRobotEnv(robot_airsim_env.RobotAirSimEnv):
    """Base class for all AirSim based uavs."""

    def __init__(self):
        rospy.loginfo('Setting up simulator environment: AirSimUAVRobotEnv.')
        # robot namespace
        self.robot_name_space = ''

        # launch connection to simulator
        super(AirSimUAVRobotEnv, self).__init__(robot_name_space=self.robot_name_space)

        self._multirotor_state = self.sim_handler.client_state

    @property
    def pose(self):
        self._multirotor_state = self.sim_handler.client_state
        airsim_position = self._multirotor_state.kinematics_estimated.position
        airsim_orientation = self._multirotor_state.kinematics_estimated.orientation
        return self.airsim_to_ros_pose(airsim_position, airsim_orientation)
    
    @property
    def velocity(self):
        airsim_lin_vel = self._multirotor_state.kinematics_estimated.linear_velocity
        airsim_ang_vel = self._multirotor_state.kinematics_estimated.angular_velocity
        return self.airsim_to_ros_twist(airsim_lin_vel, airsim_ang_vel)

    def pub_cmd_vel(self, vel_msg):
        vel_x = vel_msg.twist.linear.x
        vel_y = vel_msg.twist.linear.y
        vel_z = vel_msg.twist.linear.z
        yaw_rate = vel_msg.twist.angular.z
        self.sim_handler.client_cmd_vel(vel_x, vel_y, vel_z, yaw_rate)

    def _set_arming_request(self, arm_req, wait_time=0.05, timeout=5.0):
        return self.sim_handler.client_arm(arm_req)

    def _set_takeoff_request(self, takeoff_alt, wait_time=0.05, timeout=5.0):
        return self.sim_handler.client_takeoff
    
    def _set_land_request(self, land_alt, wait_time=0.05, timeout=5.0):
        return self.sim_handler.client_land
    
    def _check_all_systems_ready(self):
        pass
    
    def _reset_pose_estimator(self):        
        pass