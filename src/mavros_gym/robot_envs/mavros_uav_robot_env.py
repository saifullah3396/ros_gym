#!/usr/bin/env python3
"""
Defines the MavrosUAVRobotEnv class.
"""

import os
import subprocess
import rospy
from mavros_msgs.msg import State, EstimatorStatus
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from .ros_robot_env import ROSRobotEnv


class MavrosUAVRobotEnv(ROSRobotEnv):
    """
    Base class for all px4/mavros based robots.
    """

    def __init__(self):
        rospy.loginfo('Setting up simulator environment: MavrosUAVRobotEnv.')

        # robot namespace
        self.robot_name_space = ''

        # launch connection to simulator
        super(MavrosUAVRobotEnv, self).__init__()

        # set px4 pose estimator name
        est = rospy.get_param('mavros_gym/px4-est')
        if est == 'ekf2':
            self.pose_est_ = "px4-ekf2"
        elif est == 'lpe':
            self.pose_est_ = "px4-local_position_estimator"
        self.px4_ekf2_path = \
            os.path.join(
                os.environ['ROS_DEVEL'] + '/lib/px4/' + self.pose_est_)

        # subscribers
        self._state = None
        self._pose = None
        self._velocity = None
        self._gps = None
        self._est_status = None
        self.last_estimator_ts = None

        # services
        self._set_mode_client = None
        self._arming_client = None
        self._takeoff_client = None
        self._land_client = None

    def _setup_subscribers(self):
        """
        Sets up all the subscribers relating to robot state
        """
        rospy.Subscriber(
            '/mavros/state', State, callback=self._state_cb)
        rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            callback=self._pose_cb)
        rospy.Subscriber(
            '/mavros/local_position/velocity',
            TwistStamped,
            callback=self._velocity_cb)
        rospy.Subscriber(
            '/mavros/global_position/raw/fix',
            NavSatFix,
            callback=self._gps_cb)
        rospy.Subscriber(
            '/mavros/estimator_status',
            EstimatorStatus,
            callback=self._est_status_cb)

    def _state_cb(self, msg):
        self._state = msg

    def _pose_cb(self, msg):
        self._pose = msg

    def _velocity_cb(self, msg):
        self._velocity = msg

    def _gps_cb(self, msg):
        self._gps = msg

    def _est_status_cb(self, msg):
        self._est_status = msg

    @property
    def state(self):
        """ Returns the mavros state object. """
        return self._state

    @property
    def pose(self):
        """ Returns the mavros pose of the robot. """
        return self._pose

    @property
    def velocity(self):
        """ Returns the mavros velocity of the robot. """
        return self._velocity

    @property
    def gps(self):
        """ Returns the mavros gps coordinates of the robot. """
        return self._gps

    def _check_all_subscribers_ready(self):
        """
        Checks that all the subscribers are ready for connection
        """
        self._state = self._check_subscriber_ready('/mavros/state', State)
        self._pose = \
            self._check_subscriber_ready(
                '/mavros/local_position/pose', PoseStamped)
        self._velocity = \
            self._check_subscriber_ready(
                '/mavros/local_position/velocity', TwistStamped)
        self._gps = \
            self._check_subscriber_ready(
                '/mavros/global_position/raw/fix', NavSatFix)
        self._est_status = \
            self._check_subscriber_ready(
                '/mavros/estimator_status', EstimatorStatus)
        self.last_estimator_ts = \
            self._est_status.header.stamp

    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are ready for connection
        """
        self._check_publisher_ready(
            self._local_vel_pub.name, self._local_vel_pub)

    def _check_all_services_ready(self):
        """
        Checks that all the services are ready for connection
        """
        self._check_service_ready('/mavros/set_mode')
        self._check_service_ready('/mavros/cmd/arming')
        self._check_service_ready('/mavros/cmd/takeoff')
        self._check_service_ready('/mavros/cmd/land')

    def _setup_publishers(self):
        """
        Sets up all the publishers relating to robot state
        """
        # mavros publishers
        self._local_vel_pub = \
            rospy.Publisher(
                'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)

    def _pub_cmd_vel(self, vel_msg):
        self._local_vel_pub.publish(vel_msg)

    def _setup_services(self):
        # mavros services
        self._set_mode_client = \
            rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._arming_client = \
            rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._takeoff_client = \
            rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self._land_client = \
            rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    def _set_service_request(
            self, name, cond, srv, req, timeout=5.0):
        """
        Tries to set mavros px4 service requests until timeout is reached.
        """
        start_time = rospy.Time.now()
        if not cond():
            try:
                if srv(*req):
                    rospy.loginfo(
                        'Service {} request successful!'.format(name))
                    # wait for updated state
                    while not cond() and not rospy.is_shutdown():
                        if (rospy.Time.now() - start_time).to_sec() >= timeout:
                            rospy.logerr(
                                ''''Call to service {} successful but not
                                response...'''.format(name))
                            return False
                    return True
                else:
                    rospy.logwarn('Call to service %s failed.', name)
                    return False
            except rospy.ServiceException as exc:
                rospy.logwarn(
                    'Call to service {} failed with error {}'
                    .format(name, exc))
                return False
        else:
            rospy.loginfo('Service {} condition is already true.'.format(name))
            return True

    def _set_mode_request(self, mode, timeout=5.0):
        """Sets the px4 flight mode using mavros service /mavros/set_mode"""
        return self._set_service_request(
            '/mavros/set_mode',
            lambda: self._state.mode == mode,
            self._set_mode_client,
            (0, mode),  # 0 -> custom mode
            timeout
        )

    def _set_arming_request(self, arm_req, timeout=5.0):
        """
        Sets the px4 robot to arm/disarm using mavros service
        /mavros/cmd/arming
        """
        return self._set_service_request(
            '/mavros/cmd/arming',
            lambda: self._state.armed == arm_req,
            self._arming_client,
            (arm_req,),
            timeout
        )

    def _set_takeoff_request(self, takeoff_alt, timeout=5.0):
        """
        Sets the px4 robot to arm/disarm using mavros service
        /mavros/cmd/arming
        """
        return self._set_service_request(
            '/mavros/cmd/takeoff',
            lambda: self._state.mode == 'AUTO.TAKEOFF',
            self._takeoff_client,
            (0, 0, self._gps.latitude, self._gps.longitude, takeoff_alt),
            timeout
        )

    def _set_land_request(self, land_alt, timeout=5.0):
        """
        Sets the px4 robot to arm/disarm using mavros service
        /mavros/cmd/arming
        """
        return self._set_service_request(
            '/mavros/cmd/land',
            lambda: self._state.mode == 'AUTO.LAND',
            self._land_client,
            (0, 0, self._gps.latitude, self._gps.longitude, land_alt),
            timeout
        )

    def _stop_pose_estimator(self):
        ekf2_stop = subprocess.Popen([self.px4_ekf2_path, "stop"])
        ekf2_stop.wait()

    def _start_pose_estimator(self):
        ekf2_start = subprocess.Popen([self.px4_ekf2_path, "start"])
        ekf2_start.wait()

    def _check_estimator_status(self):
        while self._est_status.header.stamp == self.last_estimator_ts:
            pass
        status = \
            self._est_status.pos_horiz_rel_status_flag and \
            self._est_status.pos_horiz_abs_status_flag and \
            self._est_status.pos_horiz_rel_status_flag
        self.last_estimator_ts = self._est_status.header.stamp
        return status

    def _reset_pose_estimator(self):
        self._stop_pose_estimator()
        self._start_pose_estimator()
        rospy.loginfo("Waiting for ekf pose estimate to be corrected!")
        while True:
            if self._check_estimator_status():
                # wait for the estimator to reset
                break
