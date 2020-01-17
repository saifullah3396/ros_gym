#!/usr/bin/env python3
"""
Defines the AirsimHandler class.
"""

import sys
import rospy
import airsim
from simulation_handler import SimulationHandler
from .setup_path import SetupPath

SetupPath.add_airsim_module_path()


class AirsimHandler(SimulationHandler):
    """
    The simulation handler for airsim.
    """
    # pylint: disable=broad-except
    def __init__(self):
        super(AirsimHandler, self).__init__()
        self._new_state = True
        self._client = None
        self._multirotor_state = None

    def setup(self):
        """
        Performs initial simulation setup
        """
        self._client = airsim.MultirotorClient()
        try:
            self._client.confirmConnection()
        except Exception as _:
            rospy.logfatal(
                """Failed to connect to an airsim client.
                Please start AirSim to continue...""")
            sys.exit()

        self._client.enableApiControl(True)
        self._client.armDisarm(False)
        super(AirsimHandler, self).setup()

    def reset(self):
        """
        Resets the simulation world
        """
        try:
            self._client.reset()
            self._client.enableApiControl(True)
            self._client.armDisarm(False)
        except Exception as _:
            rospy.logerr("Failed to reset simulation.")

    def pause(self):
        """
        Pauses the simulation world
        """
        try:
            self._client.simPause(True)
        except Exception as _:
            rospy.logerr('Failed to pause simulation.')

    def unpause(self):
        """
        Unpauses the simulation world
        """
        try:
            self._client.simPause(False)
            self._new_state = True
        except Exception as _:
            rospy.logerr('Failed to unpause simulation.')

    def client_arm(self, arm_req):
        """
        Arms or disarms the robot as requested.

        Parameters
        ----------
        arm_req: bool
            Arm or disarm?
        """
        return self._client.armDisarm(arm_req)

    def client_takeoff(self, takeoff_z):
        """
        Calls the takeoff command on the robot.

        Parameters
        ----------
        takeoff_z: Float
            Height to reach on takeoff
        """
        return self._client.moveToZAsync(z=takeoff_z, velocity=0.1).join()

    def client_land(self):
        """
        Calls the land command on the robot.
        """
        return self._client.landAsync(timeout_sec=5).join()

    def client_cmd_vel(self, vel_x, vel_y, vel_z, yaw_rate):
        """
        Sends the commanded velocity to the robot.

        Parameters
        ----------
        vel_x: Float
            Linear velocity in x
        vel_y: Float
            Linear velocity in y
        vel_z: Float
            Linear velocity in z
        yaw_rate: Float
            Rotatational velocity about z

        Returns
        -------
        @todo @Hassaan: What is the return?
        """
        lin_vel_ind = \
            self._client.moveByVelocityAsync(
                vel_x, vel_y, vel_z, duration=0.005).join()
        yaw_rate_ind = \
            self._client.rotateByYawRateAsync(
                yaw_rate, duration=0.005).join()
        return lin_vel_ind and yaw_rate_ind

    @property
    def client_state(self):
        """
        Returns the state of the robot from client
        """
        if self._new_state:
            self._multirotor_state = self._client.getMultirotorState()
            self._new_state = False
        return self._multirotor_state

    @property
    def client_front_camera(self):
        """
        Returns the front camera image from the client
        """
        responses = \
            self._client.simGetImages([
                airsim.ImageRequest(
                    "0", airsim.ImageType.Scene, False, False)
                ])
        return responses[0]

    @property
    def client_collision_check(self):
        """
        Checks if the robot has collided.
        """
        return self._client.simGetCollisionInfo().has_collided
