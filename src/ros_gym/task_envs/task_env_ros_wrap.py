#!/usr/bin/env python3
"""
Defines the GymCartPoleTaskEnv class.
"""

import rospy
from ros_gym_msgs.msg import EpisodeStep
from ros_gym_msgs.srv import EnvInit, EnvStep, EnvReset


class TaskEnvRosWrap:
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

        # dictionary of space publishers
        self.space_pubs = {}

        # dictionary of state publishers
        self.state_pubs = {}

        # episode step info publisher
        self.eps_step_pub = None

        # latest action recieved through action subscriber
        self.action = None

    def setup_ros(self):
        """
        Sets up the ros wrapper by initializing publishers and subscribers for
        given observations defined in obs_types_ros_map, actions and episode
        step

        Parameters
        ----------
        state_gym: Any
            State type in gym
        """

        # add publishers for observation spaces and states
        for obs_id in self.env.observation_space.keys():
            self.setup_space_publisher(obs_id)
            self.setup_state_publisher(obs_id)

        # add a subscriber for action
        rospy.Subscriber("/action", self.action_type_ros(), self.action_cb)

        # add a publisher for stp
        self.eps_step_pub = \
            rospy.Publisher("/eps_step", EpisodeStep, queue_size=5)

        # add services for step and reset
        rospy.Service('/env_init', EnvInit, self.init_srv)
        rospy.Service('/env_step', EnvStep, self.step_srv)
        rospy.Service('/env_reset', EnvReset, self.reset_srv)

    def action_cb(self, action):
        """
        Ros callback for action subscriber
        """
        self.action = self.action_ros_to_env(action)

    # pylint: disable=unused-argument
    def init_srv(self, env_init):
        """
        Service for calling publishing initial one time stuff such as spaces
        """
        for obs_id in self.env.observation_space.keys():
            ros_space = \
                self.space_msg_env_to_ros(
                    obs_id, self.env.observation_space[obs_id])
            self.space_pubs[obs_id].publish(ros_space)

        return True

    # pylint: disable=unused-argument
    def step_srv(self, env_step):
        """
        Service for calling step function of task environment
        """
        if self.action is not None:
            state, reward, done, _ = self.env.step(self.action)
            # add publishers for observation spaces and states
            for obs_id in self.env.observation_space.keys():
                ros_state = self.state_msg_env_to_ros(obs_id, state[obs_id])
                self.state_pubs[obs_id].publish(ros_state)

            eps_step = EpisodeStep()
            eps_step.agent_id = 0
            eps_step.reward = reward
            eps_step.done = done
            self.eps_step_pub.publish()
        else:
            rospy.logwarn("Step called before action assignment.")

        self.action = None

        return True

    # pylint: disable=unused-argument
    def reset_srv(self, env_reset):
        """
        Service for calling reset function of task environment
        """
        state = self.env.reset()
        for obs_id in self.env.observation_space.keys():
            ros_state = self.state_msg_env_to_ros(obs_id, state[obs_id])
            self.state_pubs[obs_id].publish(ros_state)

        eps_step = EpisodeStep()
        eps_step.agent_id = 0
        eps_step.reward = 0
        eps_step.done = False
        self.eps_step_pub.publish()

        return True

    def setup_space_publisher(self, obs_id):
        """
        Sets up the ros publisher for space of a given observation type

        Parameters
        ----------
        obs_id: str
            Name of the observation
        """

        prefix = "/obs/" + obs_id

        # setup obs space publisher
        self.space_pubs[obs_id] = \
            rospy.Publisher(
                prefix + "/space",
                self.space_type_ros(obs_id),
                queue_size=5)
        ros_space = \
            self.space_msg_env_to_ros(
                obs_id, self.env.observation_space[obs_id])
        self.space_pubs[obs_id].publish(ros_space)

    def setup_state_publisher(self, obs_id):
        """
        Sets up the ros publisher for state of a given observation type

        Parameters
        ----------
        obs_id: str
            Name of the observation
        """

        prefix = "/obs/" + obs_id

        # setup obs publisher
        self.state_pubs[obs_id] = \
            rospy.Publisher(
                prefix, self.state_type_ros(obs_id), queue_size=5)

    def space_type_ros(self, obs_id):
        """
        Returns the ros message type for the given observation space

        Parameters
        ----------
        obs_id: str
            Name of the observation
        """

        raise NotImplementedError()

    def space_msg_env_to_ros(self, obs_id, space_gym):
        """
        Maps environment space type to ros space type for given observation

        Parameters
        ----------
        obs_id: str
            Name of the observation
        state_gym: Any
            State type in gym

        Returns
        ----------
        ros message for the given observation space
        """

        raise NotImplementedError()

    def state_type_ros(self, obs_id):
        """
        Returns the ros message type for the given observation state

        Parameters
        ----------
        obs_id: str
            Name of the observation
        """

        raise NotImplementedError()

    def state_msg_env_to_ros(self, obs_id, state_gym):
        """
        Maps environment state to ros state for given observation

        Parameters
        ----------
        obs_id: str
            Name of the observation
        state_gym: Any
            State type in gym

        Returns
        ----------
        ros message for the given observation state
        """

        raise NotImplementedError()

    def action_type_ros(self):
        """
        Returns the ros message type for the action
        """

        raise NotImplementedError()

    def action_ros_to_env(self, action_ros):
        """
        Maps ros action to environment action

        Parameters
        ----------
        action_ros: Any
            Ros message of any type

        Returns
        ----------
        Action compatible with the environment
        """

        raise NotImplementedError()