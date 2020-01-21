#!/usr/bin/env python3
"""
Defines the AirSimUAVRobotEnv class.
"""
import rospy
from gym_airsim import robot_airsim_env


class AirSimUAVRobotEnv(robot_airsim_env.RobotAirSimEnv):
    """
    Base class for all AirSim based uav robots. All common functionality
    between UAVs that also use airsim is defined here.
    """
    def __init__(self):
        rospy.loginfo('Setting up simulator environment: AirSimUAVRobotEnv.')
        super(AirSimUAVRobotEnv, self).__init__()

    @property
    def pose(self):
        """ Returns the pose of the robot from latest multirotor state. """
        multirotor_state = self.sim_handler.client_state
        airsim_position = multirotor_state.kinematics_estimated.position
        airsim_orientation = multirotor_state.kinematics_estimated.orientation
        return self.airsim_to_ros_pose(airsim_position, airsim_orientation)

    @property
    def velocity(self):
        """ Returns the velocity of the robot from latest multirotor state. """
        multirotor_state = self.sim_handler.client_state
        airsim_lin_vel = multirotor_state.kinematics_estimated.linear_velocity
        airsim_ang_vel = multirotor_state.kinematics_estimated.angular_velocity
        return self.airsim_to_ros_twist(airsim_lin_vel, airsim_ang_vel)

    def pub_cmd_vel(self, vel_msg):
        """
        Publishes the desired velocity to the robot using airsim handler.

        Parameters
        ----------
        vel_msg: TwistStamped
            Ros message for velocity
        """
        vel_x = vel_msg.twist.linear.x
        vel_y = vel_msg.twist.linear.y
        vel_z = vel_msg.twist.linear.z
        yaw_rate = vel_msg.twist.angular.z
        self.sim_handler.client_cmd_vel(vel_x, vel_y, vel_z, yaw_rate)

    def _set_arming_request(self, arm_req):
        """
        Arms/disarms the robot.

        Parameters
        ----------
        arm_req: bool
            Whether to arm or disarm?
        """
        return self.sim_handler.client_arm(arm_req)

    def _set_takeoff_request(self, takeoff_alt):
        """ Sets the takeoff request to robot. """
        return self.sim_handler.client_takeoff(takeoff_alt)

    def _set_land_request(self):
        """ Sets the land request to robot. """
        return self.sim_handler.client_land()

    def _check_all_systems_ready(self):
        """
        Checks that all connections with simulator, services, publishers, etc
        if used are operational
        """

    def _reset_pose_estimator(self):
        """
        @todo: Remove this function later. For now it is needed for the
        task_env but this function is essentially a part of mavros based envs.
        """
