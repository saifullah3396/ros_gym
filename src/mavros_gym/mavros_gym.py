#!/usr/bin/env python3
"""
Defines the ros node class MavrosGym.
"""

import rospy
import rospkg
import gym
from gym import wrappers
from gym import register
from gym import envs
from rl_agents.common.agent_base import AgentBase
from task_envs.TASK_ENV_MAP import TASK_ENV_MAP


class MavrosGym:
    """
    The base ros node for registering the custom gym environments and starting
    their training process.
    """
    def __init__(self):
        self.agent = None
        self.task_env = None

    # pylint: disable=no-self-use
    def register_env(self, task_env, max_episode_steps_per_episode=10000):
        """
        Register Gym environment. This way we can load them with variable
        limits. Here is where you have to PLACE YOUR NEW TASK ENV, to be
        registered and accesible. return: False if the Task_Env wasnt
        registered, True if it was.
        """

        name = task_env.replace("_", "-")
        if TASK_ENV_MAP[task_env] is not None:
            env_file = 'task_envs.' + task_env + ":" + TASK_ENV_MAP[task_env]
            register(
                id=name,
                entry_point=env_file,
                max_episode_steps=max_episode_steps_per_episode
            )

            # import our training environment
            # pylint: disable=exec-used
            exec(
                'from task_envs.' +
                task_env +
                ' import ' +
                TASK_ENV_MAP[task_env])

        # Check that it was really registered
        supported_gym_envs = [env_spec.id for env_spec in envs.registry.all()]
        assert (
            name in supported_gym_envs), \
            "Registration of the task_env {} failed.".format(name)
        task_env = gym.make(name)
        return task_env

    def setup(self):
        """ Gets the environment configuration and register it in gym """
        env_name = rospy.get_param('mavros_gym/environment_name')
        max_episode_steps = rospy.get_param('mavros_gym/max_episode_steps')
        self.task_env = self.register_env(env_name, max_episode_steps)

        self.agent = \
            AgentBase.get_agent(rospy.get_param('~agent'), env=self.task_env)
        rospy.loginfo('Using agent of type: {}'.format(self.agent.name))

        # Set the logging system
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mavros_gym')
        outdir = pkg_path + '/training_results'
        self.task_env = wrappers.Monitor(self.task_env, outdir, force=True)

    def start_training(self):
        """
        Starts the training process by using the specified environment
        and agent
        """
        if self.task_env is None:
            rospy.logfatal("No task environment found for training.")
        if self.agent is None:
            rospy.logfatal("No agent found for training.")
        self.agent.start_training()
