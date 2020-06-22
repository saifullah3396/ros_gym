#!/usr/bin/env python3
"""
Defines the GymCartPoleTaskEnv class.
"""

import rospy


class TaskEnvRosWrapper:
    """
    A ros wrapper for task environments for interacting with a RL agent
    over the network
    """

    def __init__(self, env):
        """
        Initializes the ros wrapper for task environment

        Parameters
        ----------
        env: The task environment
        """

        # base task environment
        self.env = env

    def setup_ros(self):
        """
        Sets up the ros wrapper by initializing services necessary for
        interaction with the underlying task environment
        """

        # add services for info, step and reset
        rospy.Service('/env_info', self.info_srv_type(), self.info_srv_cb)
        rospy.Service('/env_reset', self.reset_srv_type(), self.reset_srv_cb)
        rospy.Service('/env_step', self.step_srv_type(), self.step_srv_cb)

    # pylint: disable=unused-argument
    def info_srv_cb(self, info_input):
        """
        Service for sending info about the environment
        """

        observation_space, action_space = \
            self.space_to_ros(
                self.env.observation_space, self.env.action_space)
        return {
            "observation_space": observation_space,
            "action_space": action_space
        }

    # pylint: disable=unused-argument
    def reset_srv_cb(self, reset_input):
        """
        Service for calling reset function of task environment
        """

        state = \
            self.state_to_ros(self.env.reset())
        return {
            "state": state
        }

    # pylint: disable=unused-argument
    def step_srv_cb(self, step_input):
        """
        Service for calling step function of task environment
        """
        if step_input.action is not None:
            state, reward, done, _ = \
                self.env.step(self.action_from_ros(step_input.action))
            state = self.state_to_ros(state)

        return {
            "state": state,
            "reward": reward,
            "done": done
        }

    @classmethod
    def info_srv_type(cls):
        """
        Returns the ros service for info of underlying task environment
        """

        raise NotImplementedError()

    @classmethod
    def reset_srv_type(cls):
        """
        Returns the ros service for reset of underlying task environment
        """

        raise NotImplementedError()

    @classmethod
    def step_srv_type(cls):
        """
        Returns the ros service for step of underlying task environment
        """

        raise NotImplementedError()

    @classmethod
    def state_to_ros(cls, state_env):
        """
        Maps environment state to ros state msg
        """

        raise NotImplementedError()

    @classmethod
    def state_from_ros(cls, state_ros):
        """
        Maps ros state msg to environment state
        """

        raise NotImplementedError()

    @classmethod
    def space_to_ros(cls, observation_space_env, action_space_env):
        """
        Maps environment state to ros state msg
        """

        raise NotImplementedError()

    @classmethod
    def space_from_ros(cls, observation_space_ros, action_space_ros):
        """
        Maps ros state msg to environment state
        """

        raise NotImplementedError()

    @classmethod
    def action_type(cls):
        """
        Returns the type of action for this environment
        """

        raise NotImplementedError()

    @classmethod
    def action_to_ros(cls, action_env):
        """
        Maps environment action to ros action msg
        """

        raise NotImplementedError()

    @classmethod
    def action_from_ros(cls, action_ros):
        """
        Maps ros action msg to environment action
        """

        raise NotImplementedError()