#!/usr/bin/env python

import airsim
from simulation_handler import SimulationHandler

class AirsimHandler(SimulationHandler):
    
    def __init__(self):
        pass

    def setup(self):
        """
        Performs initial simulation setup
        """
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(False)
        
        super.setup()

    def reset(self):
        try:
            self.client.reset()
        except Exception as e:
            rospy.logerr ("Failed to reset simulation.")

    def pause(self):
        try:
            self.client.pause(True)
        except Exception as e:
            rospy.logerr('Failed to pause simulation.')
        
    def unpause(self):
        try:
            self.client.pause(False)
        except Exception as e:
            rospy.logerr('Failed to unpause simulation.')