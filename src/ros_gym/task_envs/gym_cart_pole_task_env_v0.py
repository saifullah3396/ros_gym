#!/usr/bin/env python3
"""
Defines the GymCartPoleTaskEnv class.
"""

import sys
import numpy as np
from gym.envs.classic_control import CartPoleEnv
from gym.spaces import Box, Dict, Discrete
import rospy
from ros_gym_msgs.msg import \
    BoxSpaceFloat32, CartPoleTaskState, CartPoleTaskAction
from task_envs import task_env_ros_wrap


class GymCartPoleRosWrap(task_env_ros_wrap.TaskEnvRosWrap):
    def __init__(self, env):
        super(GymCartPoleRosWrap, self).__init__(env)

    def space_type_ros(self, obs_id):
        if (obs_id == 'robot_state'):
            return BoxSpaceFloat32
        else:
            rospy.logfatal(
                "Requested observation type {} does not exist.".format(obs_id))
            sys.exit(1)
        return None

    def space_msg_env_to_ros(self, obs_id, space_gym):
        if (obs_id == 'robot_state'):
            space = BoxSpaceFloat32()
            space.low = space_gym.low
            space.high = space_gym.high
            space.shape = space_gym.shape
            return space
        else:
            rospy.logfatal(
                "Requested observation type {} does not exist.".format(obs_id))
            sys.exit(1)
        return None


    def state_type_ros(self, obs_id):
        if (obs_id == 'robot_state'):
            return CartPoleTaskState
        else:
            rospy.logfatal(
                "Requested observation type {} does not exist.".format(obs_id))
            sys.exit(1)
        return None

    def state_msg_env_to_ros(self, obs_id, state_gym):
        if (obs_id == 'robot_state'):
            state = CartPoleTaskState()
            state.cart_position = state_gym[0]
            state.cart_velocity = state_gym[1]
            state.pole_angle = state_gym[2]
            state.pole_velocity_at_tip = state_gym[3]
            return state
        else:
            rospy.logfatal(
                "Requested observation type {} does not exist.".format(obs_id))
            sys.exit(1)
        return None

    def action_type_ros(self):
        return CartPoleTaskAction

    def action_ros_to_env(self, action_ros):
        return action_ros.left_or_right


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

        ros_wrap = GymCartPoleRosWrap(self)
        ros_wrap.setup_ros()

    def step(self, action):
        state, reward, done, _ = super(GymCartPoleTaskEnv, self).step(action)
        return {"robot_state": state}, reward, done, {}

    def reset(self):
        state = super(GymCartPoleTaskEnv, self).reset()
        return {"robot_state": state}
