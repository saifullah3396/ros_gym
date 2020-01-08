#!/usr/bin/env python3
import sys
import time
import numpy as np
import rospy
import rospkg
import gym
from gym import wrappers
from gym import register
from gym import envs
from task_envs.task_env_map import task_env_map
from rl_agents.common.agent_base import AgentBase

class MavrosGym:
    def __init__(self):
        pass

    def register_env(self, task_env, max_episode_steps_per_episode=10000):
        """
        Register Gym environment. This way we can load them with variable 
        limits. Here is where you have to PLACE YOUR NEW TASK ENV, to be 
        registered and accesible. return: False if the Task_Env wasnt 
        registered, True if it was.
        """

        name = task_env.replace("_", "-")
        if task_env_map[task_env] != None:
            env_file = 'task_envs.' + task_env + ":" + task_env_map[task_env]
            register(
                id=name,
                entry_point=env_file,
                max_episode_steps=max_episode_steps_per_episode,
            )

            # import our training environment
            exec('from task_envs.' + task_env + ' import ' + task_env_map[task_env])

        # Check that it was really registered
        supported_gym_envs = [env_spec.id for env_spec in envs.registry.all()]
        assert (name in supported_gym_envs), "The task_env given is not Registered ==>" + str(name)
        task_env = gym.make(name)
        return task_env

    def setup(self):
        # Get environment configuration and register it in gym
        task_env = rospy.get_param('mavros_gym/environment_name')
        self.max_episode_steps = rospy.get_param('mavros_gym/max_episode_steps')
        task_env = self.register_env(task_env, self.max_episode_steps)

        self.agent = AgentBase.get_agent(rospy.get_param('~agent'), env=task_env)
        rospy.loginfo('Using agent of type: {}'.format(self.agent.name))

        # Set the logging system
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mavros_gym')
        outdir = pkg_path + '/training_results'
        self.task_env = wrappers.Monitor(task_env, outdir, force=True)

    def start_training(self):
        rospy.loginfo("Starting training!")
        start_time = time.time()        
        self.agent.start_training()