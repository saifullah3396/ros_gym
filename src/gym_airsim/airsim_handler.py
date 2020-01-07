#!/usr/bin/env python

import sys
import rospy
import airsim
from simulation_handler import SimulationHandler

class AirsimHandler(SimulationHandler):
    
    def __init__(self):
        super(AirsimHandler, self).__init__()
        self._new_state = True

    def setup(self):
        """
        Performs initial simulation setup
        """
        self._client = airsim.MultirotorClient()
        try:
            self._client.confirmConnection()
        except Exception as e:
            rospy.logfatal("Failed to connect to an airsim client. Please start AirSim to continue...")
            sys.exit()

        self._client.enableApiControl(True)
        self._client.armDisarm(False)
        super(AirsimHandler, self).setup()

    def reset(self):
        try:
            self._client.reset()
            self._client.enableApiControl(True)
            self._client.armDisarm(False)
        except Exception as e:
            rospy.logerr ("Failed to reset simulation.")


    def pause(self):
        try:
            self._client.simPause(True)
        except Exception as e:
            rospy.logerr('Failed to pause simulation.')


    def unpause(self):
        try:
            self._client.simPause(False)
            self._new_state = True
        except Exception as e:
            rospy.logerr('Failed to unpause simulation.')


    #AirSim Topics


    def client_arm(self, arm_req):
        return self._client.armDisarm(arm_req)


    def client_cmd_vel(self, vx, vy, vz, yaw_rate):
        lin_vel_ind = self._client.moveByVelocityAsync(vx, vy, vz,
                                                duration=0.1)
        yaw_rate_ind = self._client.rotateByYawRateAsync(yaw_rate,
                                                duration=0.1)
        return lin_vel_ind and yaw_rate_ind

    @property
    def client_state(self):
        if self._new_state:
            self._multirotor_state = self._client.getMultirotorState()
            self._new_state = False
        return self._multirotor_state

    @property
    def client_front_camera(self):
        responses = self._client.simGetImages(
                            [airsim.ImageRequest("0", airsim.ImageType.Scene,
                                                False, False)])
        return responses[0]

    @property
    def client_collision_check(self):
        collision_info = self._client.simGetCollisionInfo()
        return collision_info.has_collided

    @property
    def client_takeoff(self):
        return self._client.takeoffAsync(timeout_sec=10)
    
    @property
    def client_land(self):
        return self._client.landAsync(timeout_sec=10)