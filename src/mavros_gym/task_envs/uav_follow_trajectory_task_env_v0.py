#!/usr/bin/env python3
"""
Defines the UAVFollowTrajectoryTaskEnv class.
"""

from math import sqrt, acos, log
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TwistStamped
from robot_envs import airsim_uav_robot_env, mavros_uav_robot_env
from task_envs import uav_base_task_env

USE_MAVROS = rospy.get_param("/mavros_gym/use_mavros")
if USE_MAVROS:
    CONTROL_METHOD = mavros_uav_robot_env.MavrosUAVRobotEnv
else:
    CONTROL_METHOD = airsim_uav_robot_env.AirSimUAVRobotEnv


class UAVFollowTrajectoryTaskEnv(
        uav_base_task_env.UAVBaseTaskEnv, CONTROL_METHOD):
    """
    This class defines a task environment for reinforcement learning of UAV
    robots particularly for following a given input trajectory.
    """
    def __init__(self):
        uav_base_task_env.UAVBaseTaskEnv.__init__(self)
        CONTROL_METHOD.__init__(self)

        self.cumulated_reward = 0.0
        self.cumulated_steps = 0

    def _pre_reset(self):
        """
        Disarms the robot before resetting the simulation.
        @todo move this to mavros_uav_robot.
        """
        self.sim_handler.unpause()
        if self.use_pose_estimator:
            self._stop_pose_estimator()
            if self._set_arming_request(False):
                rospy.loginfo("Disarming successful!")
        else:
            # disarm before resetting simulation
            if self._set_arming_request(False):
                rospy.loginfo("Disarming successful!")
        # wait for robot to fall
        rospy.sleep(2.0)

    def _set_init_pose(self):
        """
        Sets the initial state of the robot as required.
        """

        # set the linear and angular velocities to execute
        # self.pub_cmd_vel(self.init_velocity)
        return True

    def _init_env_variables(self):
        """
        Initializes the environment for a new episode run.
        """
        self.sim_handler.unpause()
        if self.use_pose_estimator:
            self._reset_pose_estimator()
        self._check_all_systems_ready()
        if self._set_arming_request(True):
            rospy.loginfo("Arming successful!")
        if self._set_takeoff_request(1):
            rospy.loginfo("Takeoff successful!")

        # for information
        self.cumulated_reward = 0.0
        self.cumulated_steps = 0

        # we get the initial pose to measure the distance from
        # the desired point.
        curr_pose = self.pose

        # pylint: disable=attribute-defined-outside-init
        self.previous_distance_from_des_point = \
            self.get_distance_from_desired_point(curr_pose.pose.position)

        self.previous_difference_from_des_orientation = \
            self.get_difference_from_desired_orientation(
                curr_pose.pose.orientation)

    def _set_action(self, action):
        """
        Sets the action in the form of linear/angular velocities send to the
        robot.

        Parameters
        ----------
        action: np.array
            A numpy array of size 4 = [vel_x, vel_y, vel_z, yaw]
        """
        action_vel = TwistStamped()
        action_vel.twist.linear.x = action[0]
        action_vel.twist.linear.y = action[1]
        action_vel.twist.linear.z = action[2]
        action_vel.twist.angular.x = 0.0
        action_vel.twist.angular.y = 0.0
        action_vel.twist.angular.z = action[3]

        # set the desired velocity by publishing it to the robot
        self.pub_cmd_vel(action_vel)

    def _get_obs(self):
        """
        Returns the current environment observation. In this environment, a
        dictionary of position, velocity, and front_cam image is returned. The
        returned data must conform with env.observation_space. See
        UAVBaseTaskEnv for more info.
        """
        curr_pose = self.pose
        curr_vel = self.velocity
        pos_obs_obs = \
            np.array([
                curr_pose.pose.position.x,
                curr_pose.pose.position.y,
                curr_pose.pose.position.z,
                curr_pose.pose.orientation.w,
                curr_pose.pose.orientation.x,
                curr_pose.pose.orientation.y,
                curr_pose.pose.orientation.z])
        vel_obs_space = \
            np.array([
                curr_vel.twist.linear.x,
                curr_vel.twist.linear.y,
                curr_vel.twist.linear.z,
                curr_vel.twist.angular.x,
                curr_vel.twist.angular.y,
                curr_vel.twist.angular.z])
        return {
            "position": pos_obs_obs,
            "velocity": vel_obs_space,
            "front_cam": self.front_camera
        }

    def _is_done(self, observations):
        """
        Returns true if the environment needs to be reset based on the
        following constraints:
        1) The robot has collided with the environment.
        2) The robot has gone outside the workspace
        3) The robot has detected something too close
        4) The robot has flipped or crashed.
        5) The robot has reached its desired destination.

        Parameters
        ----------
        observations: Observation of the type defined in observation_space.
        """

        episode_done = False
        current_pose = observations['position']
        current_position = observations['position'][:3]
        current_orientation = observations['position'][3:7]

        if self.collision_check:
            rospy.loginfo(
                '''Episode finished due to robot collision.''')
            return True

        if not self.is_inside_workspace(current_position):
            rospy.loginfo(
                '''Episode finished since the robot has gone outside
                the workspace.''')
            return True

        if self.too_close_to_ground(-1*current_position[2]):
            rospy.loginfo(
                '''Episode finished since the robot has gone too close to
                the ground.''')
            return True

        if self.drone_has_flipped(current_orientation):
            rospy.loginfo(
                'Episode finished since the robot has flipped.')
            return True

        if self.is_in_desired_pose(current_pose, self.desired_pose_epsilon):
            rospy.loginfo(
                '''Episode finished since the robot has successfully reached
                its destination.''')
            return True

        return episode_done

    def _compute_reward(self, observations, done):
        """
        Defines the reward function for this environment.
        """
        current_pose = PoseStamped()
        current_pose.pose.position.x = observations['position'][0]
        current_pose.pose.position.y = observations['position'][1]
        current_pose.pose.position.z = observations['position'][2]
        current_pose.pose.orientation.w = observations['position'][3]
        current_pose.pose.orientation.x = observations['position'][4]
        current_pose.pose.orientation.y = observations['position'][5]
        current_pose.pose.orientation.z = observations['position'][6]

        distance_from_des_point = \
            self.get_distance_from_desired_point(current_pose.pose.position)
        difference_from_des_orientation = \
            self.get_difference_from_desired_orientation(
                current_pose.pose.orientation)
        distance_difference = \
            distance_from_des_point - \
            self.previous_distance_from_des_point + \
            2 * (
                difference_from_des_orientation -
                self.previous_difference_from_des_orientation)

        if not done:
            if self.collision_check:
                reward = self.collision_check
            # if there has been a decrease in the distance to the desired
            # location, we reward it
            if distance_difference < 0.0:
                rospy.loginfo(
                    '''Robot rewarded for getting close to the desired
                    destination.''')
                reward = self.closer_to_point_reward
            else:
                rospy.loginfo(
                    '''Robot unrewarded for getting away from the desired
                    destination.''')
                reward = 0
        else:
            if self.collision_check:
                reward = self.collision_penalty
            elif self.is_in_desired_pose(
                    observations['position'][:7], tolerance=0.5):
                reward = self.end_episode_points
            else:
                reward = -1*self.end_episode_points

        self.previous_distance_from_des_point = distance_from_des_point
        self.previous_difference_from_des_orientation = \
            difference_from_des_orientation

        self.cumulated_reward += reward
        self.cumulated_steps += 1
        rospy.logdebug("Reward = {}".format(reward))
        rospy.logdebug("Cumulated reward = {}".format(self.cumulated_reward))
        rospy.logdebug("Cumulated steps = {}".format(self.cumulated_steps))

        return reward

    def is_in_desired_pose(self, current_pose, tolerance=0.05):
        """
        Returns true if the current position is close to the desired position

        Parameters
        ----------
        current_pose: np.array
            Current pose of the robot
        tolerance: Float
            How far the robot can be from the desired destination pose
        """

        current_pose_array = np.asarray(current_pose)
        desired_pose = \
            np.array([
                self.desired_pose.pose.position.x,
                self.desired_pose.pose.position.y,
                self.desired_pose.pose.position.z,
                self.desired_pose.pose.orientation.w,
                self.desired_pose.pose.orientation.x,
                self.desired_pose.pose.orientation.y,
                self.desired_pose.pose.orientation.z])

        desired_pose_plus = desired_pose + tolerance
        desired_pose_minus = desired_pose - tolerance

        return \
            np.all(current_pose_array <= desired_pose_plus) and \
            np.all(current_pose_array > desired_pose_minus)

    def is_inside_workspace(self, current_position):
        """
        Returns true if the robot is inside the workspace bounds.
        """
        return all(
            [
                self.work_space_x_min <= current_position[0] <=
                self.work_space_x_max,
                self.work_space_y_min <= current_position[1] <=
                self.work_space_y_max,
                self.work_space_z_min <= current_position[2] <=
                self.work_space_z_max,
            ]
        )

    def too_close_to_ground(self, current_position_z):
        """
        Returns true if the robot is too close to the ground.
        """
        return current_position_z < self.min_height

    def drone_has_flipped(self, current_orientation):
        """
        Returns true if the robot has flipped.
        """
        curr_roll, curr_pitch, _ = \
            euler_from_quaternion([
                current_orientation[1],
                current_orientation[2],
                current_orientation[3],
                current_orientation[0]])
        self.max_roll = rospy.get_param("/mavros_gym/max_roll")
        self.max_pitch = rospy.get_param("/mavros_gym/max_pitch")

        return not all(
            [
                -1*self.max_roll <= curr_roll <= self.max_roll,
                -1*self.max_pitch <= curr_pitch <= self.max_pitch
            ])

    def get_distance_from_desired_point(self, current_position):
        """
        Returns the distance between the current position and desired
        position
        """
        curr_position = \
            np.array([
                current_position.x, current_position.y, current_position.z])
        des_position = \
            np.array([
                self.desired_pose.pose.position.x,
                self.desired_pose.pose.position.y,
                self.desired_pose.pose.position.z])
        return np.linalg.norm(curr_position - des_position)

    def get_difference_from_desired_orientation(self, current_orientation):
        """
        Calculates the distance from the current orientation and the desired
        orientation.
        """
        curr_orientation = \
            np.array([
                current_orientation.w,
                current_orientation.x,
                current_orientation.y,
                current_orientation.z])
        des_orientation = \
            np.array([
                self.desired_pose.pose.orientation.w,
                self.desired_pose.pose.orientation.x,
                self.desired_pose.pose.orientation.y,
                self.desired_pose.pose.orientation.z])
        difference = \
            self.get_difference_between_orientations(
                curr_orientation, des_orientation)
        return difference

    def get_difference_between_orientations(self, o_start, o_end):
        """
        Returns the difference between two orientations.
        """

        if self.geo_distance is True:
            # geodesic distance
            if np.dot(o_start, o_start) > 0:
                o_start_conj = \
                    np.array((o_start[0], -1 * o_start[1:4])) / \
                    np.dot(o_start, o_start)
            else:
                rospy.logerr(
                    """can not compute the orientation difference of a
                    quaternion with 0 norm""")
                return float('NaN')

            o_product = o_start_conj * o_end
            o_product_vector = o_product[1:4]

            v_product_norm = np.linalg.norm(o_product_vector)
            o_product_norm = sqrt(np.dot(o_product, o_product))

            tolerance = 1e-17
            if o_product_norm < tolerance:
                # 0 quaternion - undefined
                o_diff = \
                    np.array([-float('inf'), float('nan')*o_product_vector])
            if v_product_norm < tolerance:
                # real quaternions - no imaginary part
                o_diff = np.array([log(o_product_norm), 0, 0, 0])
            vec = o_product_vector / v_product_norm
            o_diff = \
                np.array(
                    log(o_product_norm),
                    acos(o_product[0] / o_product_norm)*vec)

            difference = sqrt(np.dot(o_diff, o_diff))
            return difference

        else:
            # absolute distance
            o_start_minus_o_end = o_start - o_end
            o_start_plus_o_end = o_start + o_end
            d_minus = sqrt(np.dot(o_start_minus_o_end, o_start_minus_o_end))
            d_plus = sqrt(np.dot(o_start_plus_o_end, o_start_plus_o_end))
            if d_minus < d_plus:
                return d_minus
            else:
                return d_plus
