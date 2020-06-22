#!/usr/bin/env python3
"""
Defines the GymCartPoleTaskEnv class.
"""

import numpy as np
from gym.envs.classic_control import CartPoleEnv
from gym.spaces import Box, Discrete
from ros_gym_msgs.msg import DiscreteSpace, BoxSpaceFloat32
from cart_pole_env_ros_msgs.msg import \
    CartPoleTaskSpace, CartPoleTaskState, CartPoleTaskAction
from cart_pole_env_ros_msgs.srv import \
    CartPoleEnvInfo, CartPoleEnvReset, CartPoleEnvStep
from .. import task_env_ros_wrapper


class GymCartPoleRosWrapper(task_env_ros_wrapper.TaskEnvRosWrapper):
    def __init__(self, env):
        super(GymCartPoleRosWrapper, self).__init__(env)

    @classmethod
    def info_srv_type(cls):
        return CartPoleEnvInfo

    @classmethod
    def reset_srv_type(cls):
        return CartPoleEnvReset

    @classmethod
    def step_srv_type(cls):
        return CartPoleEnvStep

    @classmethod
    def space_to_ros(cls, observation_space_env, action_space_env):
        observation_space_ros = CartPoleTaskSpace()
        observation_space_ros.robot_state.low = \
            observation_space_env['robot_state'].low
        observation_space_ros.robot_state.high = \
            observation_space_env['robot_state'].high
        observation_space_ros.robot_state.shape = \
            observation_space_env['robot_state'].shape

        action_space_ros = DiscreteSpace()
        action_space_ros.n = action_space_env.n

        return observation_space_ros, action_space_ros

    @classmethod
    def space_from_ros(cls, observation_space_ros, action_space_ros):
        observation_space_env = {}
        observation_space_env['robot_state'] = \
            Box(
                low=np.array(observation_space_ros.robot_state.low),
                high=np.array(observation_space_ros.robot_state.high),
                shape=np.array(observation_space_ros.robot_state.shape))

        action_space_env = Discrete(action_space_ros.n)

        return observation_space_env, action_space_env

    @classmethod
    def state_to_ros(cls, state_env):
        # create a state msg
        state = CartPoleTaskState()

        # assign state variables
        state.robot_state.cart_position = state_env['robot_state'][0]
        state.robot_state.cart_velocity = state_env['robot_state'][1]
        state.robot_state.pole_angle = state_env['robot_state'][2]
        state.robot_state.pole_velocity_at_tip = state_env['robot_state'][3]
        return state

    @classmethod
    def state_from_ros(cls, state_ros):
        state_env = []
        state_env.append(state_ros.robot_state.cart_position)
        state_env.append(state_ros.robot_state.cart_velocity)
        state_env.append(state_ros.robot_state.pole_angle)
        state_env.append(state_ros.robot_state.pole_velocity_at_tip)
        return {'robot_state': state_env}

    @classmethod
    def action_type(cls):
        return CartPoleTaskAction

    @classmethod
    def action_from_ros(cls, action_ros):
        return action_ros.left_or_right

    @classmethod
    def action_to_ros(cls, action):
        action_ros = CartPoleTaskAction()
        action_ros.left_or_right = action
        return action_ros


class GymCartPoleTaskEnv(CartPoleEnv):
    """
    This class wraps the gym cartpole environment.
    """
    def __init__(self):
        super(GymCartPoleTaskEnv, self).__init__()
        # Angle limit set to 2 * theta_threshold_radians so failing obs
        # is still within bounds
        high = np.array([
            self.x_threshold * 2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians * 2,
            np.finfo(np.float32).max])

        self.observation_space = {}
        self.observation_space['robot_state'] = \
            Box(-high, high, dtype=np.float32)

        ros_wrapper = GymCartPoleRosWrapper(self)
        ros_wrapper.setup_ros()

    def step(self, action):
        state, reward, done, _ = super(GymCartPoleTaskEnv, self).step(action)
        return {'robot_state': state}, reward, done, {}

    def reset(self):
        state = super(GymCartPoleTaskEnv, self).reset()
        return {'robot_state': state}
