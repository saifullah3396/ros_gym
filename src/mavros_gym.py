#!/usr/bin/env python

import rospy
import rospkg

import gym
from gym.envs.registration import register
from gym import wrappers
from gym import envs
from task_envs import uav_follow_trajectory_task_env

import numpy as np

class MavrosGym:
    def __init__(self):
        pass

    def setup(self):
        # Get environment configuration and register it in gym
        task_env = rospy.get_param('mavros_gym/environment_name')
        self.max_episode_steps = rospy.get_param('mavros_gym/max_episode_steps')
        task_env = self.register_env(task_env, self.max_episode_steps)
        # Set the logging system
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mavros_gym')
        outdir = pkg_path + '/training_results'
        self.task_env = wrappers.Monitor(task_env, outdir, force=True)

    def register_env(self, task_env, max_episode_steps=10000):
        # register env to gym
        if task_env == 'uav-follow-trajectory-env-v0':
            register(
                id=task_env,
                entry_point='task_envs.uav_follow_trajectory_task_env:UAVFollowTrajectoryTaskEnv',
                max_episode_steps=max_episode_steps,
            )
        
        # Check that it was really registered
        supported_gym_envs = [env_spec.id for env_spec in envs.registry.all()]
        assert (task_env in supported_gym_envs), "The task_env given is not Registered ==>" + str(task_env)
        task_env = gym.make(task_env)
        return task_env

    def start_training(self):
        rospy.loginfo("Starting training!")
        #last_time_steps = np.ndarray(0)
        #n_eps = rospy.get_param("/mavros_gym/n_eps")
        #start_time = time.time()
        #highest_reward = 0
        observation = self.task_env.reset()
    
        # Starts the main training loop: the one about the episodes to do
        '''for i in range(n_eps):
        rospy.loginfo("############### START EPISODE=>" + str(x))
        observation = env.reset()
        state = ''.join(map(str, observation))
        rospy.logwarn(state)
        cumulated_reward = 0
        done = False
        for i in range(nsteps):
        rospy.loginfo("Episode: {0} step: {1}".format(x,i))
        time.sleep(20)
        env_to_wrap.close()'''