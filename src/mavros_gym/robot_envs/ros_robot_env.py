#!/usr/bin/env python3
"""
Defines the ROSRobotEnv class.
"""

import rospy
from rospy import ROSException
from gym_gazebo import robot_gazebo_env
from gym_airsim import robot_airsim_env

SIM_ENV = rospy.get_param("/mavros_gym/sim_env")
if SIM_ENV == 'gazebo':
    SIMULATION_ENV = robot_gazebo_env.RobotGazeboEnv
elif SIM_ENV == 'airsim':
    SIMULATION_ENV = robot_airsim_env.RobotAirSimEnv
else:
    raise NotImplementedError(
        'Simulation environment ' + SIM_ENV + ' not supported.')


class ROSRobotEnv(SIMULATION_ENV):
    """
    Defines the base environmnet for simulation of robot of any type.
    """
    def __init__(self):
        # robot namespace
        self.robot_name_space = ''

        # launch connection to gazebo
        if SIM_ENV == 'gazebo':
            super(ROSRobotEnv, self).__init__(
                robot_name_space=self.robot_name_space,
                update_physics_params_at_start=True)
        elif SIM_ENV == 'airsim':
            super(ROSRobotEnv, self).__init__(
                robot_name_space=self.robot_name_space)

        self.sim_handler.unpause()
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        self._check_all_systems_ready()
        self._setup_services()
        self.sim_handler.pause()

    def _check_all_systems_ready(self):
        """
        Checks that all the subscribers, publishers, services and other
        simulation systems are operational.
        """
        self._check_all_subscribers_ready()
        self._check_all_publishers_ready()
        self._check_all_services_ready()
        return True

    def _check_all_subscribers_ready(self):
        """
        Checks that all the subscribers are ready for connection
        """
        raise NotImplementedError()

    def _check_all_publishers_ready(self):
        """
        Checks that all the sensors are ready for connection
        """
        raise NotImplementedError()

    def _setup_subscribers(self):
        """
        Sets up all the subscribers relating to robot state
        """
        raise NotImplementedError()

    def _setup_publishers(self):
        """
        Sets up all the publishers relating to robot state
        """
        raise NotImplementedError()

    def _check_subscriber_ready(self, name, srv_type, timeout=5.0):
        """
        Waits for a sensor topic to get ready for connection
        """
        var = None
        while var is None and not rospy.is_shutdown():
            try:
                var = rospy.wait_for_message(name, srv_type, timeout)
            except ROSException:
                rospy.logerr(
                    '''Sensor topic {} is not available.
                    Waiting...'''.format(name))
        return var

    def _check_publisher_ready(self, name, obj, timeout=5.0):
        """
        Waits for a publisher to get response
        """
        start_time = rospy.Time.now()
        while obj.get_num_connections() == 0 and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() >= timeout:
                rospy.logerr(
                    '''No subscriber found for the publisher {}.
                    Exiting...'''.format(name))

    def _check_service_ready(self, name, timeout=5.0):
        """
        Waits for a service to get ready
        """
        try:
            rospy.wait_for_service(name, timeout)
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logerr("Service {} unavailable.".format(name))
