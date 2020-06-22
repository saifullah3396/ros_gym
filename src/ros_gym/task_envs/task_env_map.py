#!/usr/bin/env python3
"""
This files defines the task map for all the environments that are possible. Any
new environment must be defined here as <filename>: <classname>
"""


TASK_ENV_MAP = {
    'uav_follow_trajectory_task_env_v0': 'UAVFollowTrajectoryTaskEnv',
    'gym_cart_pole_task_env_v0': 'GymCartPoleTaskEnv',
    'gym_mc_continuous_task_env_v0': 'GymMCContinuousTaskEnv'
}

TASK_ENV_ROS_MAP = {
    'gym_cart_pole_task_env_v0': 'GymCartPoleRosWrapper',
}
