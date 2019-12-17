import rospy
import gym
from gym.utils import seeding
from mavros_gym_msgs.msg import RLExperimentInfo

class RobotSimEnv(gym.Env):

    def __init__(self, robot_name_space, sim_handler):
        
        self.sim_handler = sim_handler
        self.episode_num = 0
        self.cumulated_episode_reward = 0
        self.reward_pub = rospy.Publisher('/openai/reward', RLExperimentInfo, queue_size=1)

    def step(self, action):
        """
        Function executed each time step.
        Here we get the action execute it in a time step and retrieve the
        observations generated by that action.
        :param action:
        :return: obs, reward, done, info
        """

        self.sim_handler.unpause()
        self._set_action(action)
        self.sim_handler.pause()
        obs = self._get_obs()
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward
        return obs, reward, done, info

    def reset(self):
        self._pre_reset()
        self.sim_handler.pause()
        self.sim_handler.reset()
        self._set_init_pose()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()
        return obs

    def close(self):
        pass

    def _update_episode(self):
        """
        Publishes the cumulated reward of the episode and
        increases the episode number by one.
        :return:
        """
        self._publish_reward_topic(self.cumulated_episode_reward, self.episode_num)
        self.episode_num += 1
        self.cumulated_episode_reward = 0

    def _publish_reward_topic(self, reward, episode_number=1):
        """
        This function publishes the given reward in the reward topic for
        easy access from ROS infrastructure.
        :param reward:
        :param episode_number:
        :return:
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self.reward_pub.publish(reward_msg)

    def _pre_reset():
        """Performs operations required prior to simulation state reset
        """
        raise NotImplementedError()

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Returns the observation.
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """Indicates whether or not the episode is done ( the robot has fallen for example).
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        raise NotImplementedError()

