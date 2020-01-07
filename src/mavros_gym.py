#!/usr/bin/env python3
import sys
import time
import numpy as np
import rospy
import rospkg
import gym
from gym import wrappers
from make_gym_env import GymMake
from ddpg_conv_state.agent import Agent

class MavrosGym:
    def __init__(self):
        pass
    
    def setup(self):
        # Get environment configuration and register it in gym
        task_env = rospy.get_param('mavros_gym/environment_name')
        self.max_episode_steps = rospy.get_param('mavros_gym/max_episode_steps')
        task_env = GymMake(task_env, self.max_episode_steps)
        self.agent = Agent(env=task_env)
        # Set the logging system
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mavros_gym')
        outdir = pkg_path + '/training_results'
        self.task_env = wrappers.Monitor(task_env, outdir, force=True)

    def start_training(self):
        rospy.loginfo("Starting training!")
        start_time = time.time()
        self.agent.start_training()