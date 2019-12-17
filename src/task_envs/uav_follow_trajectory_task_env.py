#!/usr/bin/env python
import numpy as np
import rospy
from math import sqrt, pi, cos, acos, log
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped, Quaternion
from robot_envs import mavros_uav_robot_env
from task_envs import uav_base_task_env


class UAVFollowTrajectoryTaskEnv(uav_base_task_env.UAVBaseTaskEnv, mavros_uav_robot_env.MavrosUAVRobotEnv):
    def __init__(self):    
        """
        Make a mavros based drone learn how to follow a trajectory
        """
        uav_base_task_env.UAVBaseTaskEnv.__init__(self)
        mavros_uav_robot_env.MavrosUAVRobotEnv.__init__(self)

    def _pre_reset(self):
        self.sim_handler.unpause()
        if self.use_pose_estimator:
            self._stop_pose_estimator()
            if (self._set_arming_request(False)):
                rospy.loginfo("Disarming successful!")
        else:
            # disarm before resetting simulation
            if (self._set_arming_request(False)):
                rospy.loginfo("Disarming successful!")
        rospy.sleep(2.0) # wait for robot to fall

    def _set_init_pose(self):
        """
        Sets the Robot in its init linear and angular speeds.
        Its preparing it to be reseted in the world.
        """
        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.sim_handler.unpause()
        if self.use_pose_estimator:
            self._reset_pose_estimator()
        self._check_all_systems_ready()
        if (self._set_arming_request(True)):
            rospy.loginfo("Arming successful!")
        if (self._set_takeoff_request(1)):
            rospy.loginfo("Takeoff successful!")

        # For Info Purposes
        self.cumulated_reward = 0.0

        # We get the initial pose to measure the distance from the desired point.
        curr_pose = self.pose
        self.previous_distance_from_des_point = \
            self.get_distance_from_desired_point(curr_pose.pose.position)

        self.previous_difference_from_des_orientation = \
            self.get_difference_from_desired_orientation(curr_pose.pose.orientation)

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the drone
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
        action_vel = TwistStamped()
        action_vel.twist.linear.x   = action[0]
        action_vel.twist.linear.y   = action[1]
        action_vel.twist.linear.z   = action[2]
        action_vel.twist.angular.x  = 0.0
        action_vel.twist.angular.y  = 0.0
        action_vel.twist.angular.z  = action[3]
        
        # We tell drone the linear and angular velocities to set to execute
        self.ExecuteAction(action_vel)
        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        droneEnv API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        curr_pose = self.pose

        observations = [curr_pose.pose.position.x,\
                        curr_pose.pose.position.y,\
                        curr_pose.pose.position.z,\
                        curr_pose.pose.orientation.w,\
                        curr_pose.pose.orientation.x,\
                        curr_pose.pose.orientation.y,\
                        curr_pose.pose.orientation.z]

        rospy.logdebug("Observations==>"+str(observations))
        rospy.logdebug("END Get Observation ==>")
        return observations

    def _is_done(self, observations):
        """
        The done can be done due to three reasons:
        1) It went outside the workspace
        2) It detected something with the sonar that is too close
        3) It flipped due to a crash or something
        4) It has reached the desired point
        """

        episode_done = False
        current_pose = observations
        current_position = observations[:3]
        current_orientation = observations[3:]

        is_inside_workspace_now = self.is_inside_workspace(current_position)
        too_close_to_grnd       = self.too_close_to_ground(current_position[2])
        drone_flipped           = self.drone_has_flipped(current_orientation)
        has_reached_des_pose    = self.is_in_desired_pose(current_pose, self.desired_pose_epsilon)

        rospy.logwarn(">>>>>> DONE RESULTS <<<<<")

        if not is_inside_workspace_now:
            rospy.logerr("is_inside_workspace_now=" +
                         str(is_inside_workspace_now))
        else:
            rospy.logwarn("is_inside_workspace_now=" +
                          str(is_inside_workspace_now))

        if too_close_to_grnd:
            rospy.logerr("too_close_to_ground=" + str(too_close_to_grnd))
        else:
            rospy.logwarn("too_close_to_ground=" + str(too_close_to_grnd))

        if drone_flipped:
            rospy.logerr("drone_flipped="+str(drone_flipped))
        else:
            rospy.logwarn("drone_flipped="+str(drone_flipped))

        if has_reached_des_pose:
            rospy.logerr("has_reached_des_pose="+str(has_reached_des_pose))
        else:
            rospy.logwarn("has_reached_des_pose="+str(has_reached_des_pose))

        # We see if we are outside the Learning Space
        episode_done = not(is_inside_workspace_now) or\
                        too_close_to_grnd or\
                        drone_flipped or\
                        has_reached_des_pose

        if episode_done:
            rospy.logerr("episode_done====>"+str(episode_done))
        else:
            rospy.logwarn("episode running! \n")

        return episode_done

    def _compute_reward(self, observations, done):
        
        current_pose = PoseStamped()
        current_pose.pose.position.x    = observations[0]
        current_pose.pose.position.y    = observations[1]
        current_pose.pose.position.z    = observations[2]
        current_pose.pose.orientation.w = observations[3]
        current_pose.pose.orientation.x = observations[4]
        current_pose.pose.orientation.y = observations[5]
        current_pose.pose.orientation.z = observations[6]
        

        distance_from_des_point = self.get_distance_from_desired_point(current_pose.pose.position)
        
        difference_from_des_orientation = self.get_difference_from_desired_orientation(current_pose.pose.orientation)
        
        distance_difference = distance_from_des_point - self.previous_distance_from_des_point + \
                                2*(difference_from_des_orientation - self.previous_difference_from_des_orientation)

        if not done:

            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logwarn("DECREASE IN DISTANCE GOOD")
                reward = self.closer_to_point_reward
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                reward = 0

        else:

            if self.is_in_desired_pose(current_position, epsilon=0.5):
                reward = self.end_episode_points
            else:
                reward = -1*self.end_episode_points

        self.previous_distance_from_des_point = distance_from_des_point
        self.previous_difference_from_des_orientation = difference_from_des_orientation

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward

    # Internal TaskEnv Methods

    def is_in_desired_pose(self, current_pose, epsilon=0.05):
        """
        It return True if the current position is similar to the desired poistion
        """

        is_in_desired_pose = False
        current_pose = np.array(current_pose)
        desired_pose = np.array([self.desired_pose.pose.position.x,\
                        self.desired_pose.pose.position.y,\
                        self.desired_pose.pose.position.z,\
                        self.desired_pose.pose.orientation.w,\
                        self.desired_pose.pose.orientation.x,\
                        self.desired_pose.pose.orientation.y,\
                        self.desired_pose.pose.orientation.z])
        
        desired_pose_plus = desired_pose + epsilon
        desired_pose_minus= desired_pose - epsilon

        pose_are_close = np.all(current_pose <= desired_pose_plus) and \
                         np.all(current_pose >  desired_pose_minus)



        rospy.logwarn("###### IS DESIRED POS ? ######")

        rospy.logwarn("current_pose"+str(current_pose))

        rospy.logwarn("desired_pose_plus"+str(desired_pose_plus) +\
                      ",desired_pose_minus="+str(desired_pose_minus))

        rospy.logwarn("pose_are_close"+str(pose_are_close))
        rospy.logwarn("is_in_desired_pose"+str(is_in_desired_pos))

        rospy.logwarn("############")

        return is_in_desired_pose

    def is_inside_workspace(self, current_position):
        """
        Check if the Drone is inside the Workspace defined
        """
        is_inside = False

        rospy.logwarn("##### INSIDE WORK SPACE? #######")
        rospy.logwarn("XYZ current_position"+str(current_position))
        rospy.logwarn("work_space_x_max"+str(self.work_space_x_max) +
                      ",work_space_x_min="+str(self.work_space_x_min))
        rospy.logwarn("work_space_y_max"+str(self.work_space_y_max) +
                      ",work_space_y_min="+str(self.work_space_y_min))
        rospy.logwarn("work_space_z_max"+str(self.work_space_z_max) +
                      ",work_space_z_min="+str(self.work_space_z_min))
        rospy.logwarn("############")

        if current_position[0] > self.work_space_x_min and current_position[0] <= self.work_space_x_max:
            if current_position[1] > self.work_space_y_min and current_position[1] <= self.work_space_y_max:
                if current_position[2] > self.work_space_z_min and current_position[2] <= self.work_space_z_max:
                    is_inside = True

        return is_inside

    def too_close_to_ground(self, current_position_z):
        """
        Detects if there is something too close to the drone front
        """
        rospy.logwarn("##### SONAR TOO CLOSE? #######")
        rospy.logwarn("Current height"+str(current_position_z) +
                      ",min_allowed_height="+str(self.min_height))
        rospy.logwarn("############")

        too_close = sonar_value < self.min_height

        return too_close

    def drone_has_flipped(self, current_orientation):
        """
        Based on the orientation RPY given states if the drone has flipped
        """
        has_flipped = True

        curr_roll, curr_pitch, curr_yaw = euler_from_quaternion([current_orientation[1],\
                                                                 current_orientation[2],\
                                                                 current_orientattion[3],\
                                                                 current_orientation[0]])
        self.max_roll = rospy.get_param("/mavros_gym/max_roll")
        self.max_pitch = rospy.get_param("/mavros_gym/max_pitch")

        rospy.logwarn("#### HAS FLIPPED? ########")
        rospy.logwarn("RPY current_orientation"+str(curr_roll, curr_pitch, curr_yaw))
        rospy.logwarn("max_roll"+str(self.max_roll) +
                      ",min_roll="+str(-1*self.max_roll))
        rospy.logwarn("max_pitch"+str(self.max_pitch) +
                      ",min_pitch="+str(-1*self.max_pitch))
        rospy.logwarn("############")

        if curr_roll > -1*self.max_roll and curr.roll <= self.max_roll:
            if curr_pitch > -1*self.max_pitch and curr_pitch <= self.max_pitch:
                has_flipped = False

        return has_flipped

    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        curr_position = np.array([current_position.x, current_position.y, current_position.z])
        des_position = np.array([self.desired_pose.pose.position.x,\
                                self.desired_pose.pose.position.y,\
                                self.desired_pose.pose.position.z])
        distance = self.get_distance_between_points(curr_position, des_position)

        return distance

    def get_distance_between_points(self, p_start, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = np.array(p_start)
        b = np.array(p_end)

        distance = np.linalg.norm(a - b)

        return distance

    def get_difference_from_desired_orientation(self, current_orientation):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        curr_orientation = np.array([current_orientation.w, current_orientation.x, current_orientation.y, current_orientation.z])
        des_orientation = np.array([self.desired_pose.pose.orientation.w,\
                                self.desired_pose.pose.orientation.x,\
                                self.desired_pose.pose.orientation.y,\
                                self.desired_pose.pose.orientation.z])
        difference = self.get_difference_between_orientations(curr_orientation, des_orientation)

        return difference
    
    def get_difference_between_orientations(self, ostart, o_end):
        """
        Given an orientation Object, get difference from current orientation
        :param p_end:
        :return:
        """

        if self.geo_distance == True:   #<-- Geodesic distance
            if np.dot(ostart, ostart) > 0:
                ostart_conj = np.array((ostart[0], -1*ostart[1:4])) / np.dot(ostart, ostart)
            else:
                rospy.logerr("can not compute the orientation difference of a quaternion with 0 norm")
                return float('NaN')
        
            o_product = ostart_conj * o_end
            o_product_vector = o_product[1:4]
        
            v_product_norm = np.linalg.norm(o_product_vector)
            o_product_norm = sqrt(np.dot(o_product, o_product))

            tolerance = 1e-17
            if o_product_norm < tolerance:
                # 0 quaternion - undefined
                o_diff = np.array([-float('inf'), float('nan')*o_product_vector])
            if v_product_norm < tolerance:
                # real quaternions - no imaginary part
                o_diff = np.array([log(o_product_norm),0,0,0])
            vec = o_product_vector / v_product_norm
            o_diff = np.array(log(o_product_norm), acos(o_product[0]/o_product_norm)*vec)

            difference = sqrt(np.dot(o_diff, o_diff))
            return difference

        else: #<-- Absolute distance
            ostart_minus_o_end = ostart - o_end
            ostart_plus_o_end  = ostart + o_end
            d_minus = sqrt(np.dot(ostart_minus_o_end, ostart_minus_o_end))
            d_plus  = sqrt(np.dot(ostart_plus_o_end, ostart_plus_o_end))
            if (d_minus < d_plus):
                return d_minus
            else:
                return d_plus

    """ def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw """