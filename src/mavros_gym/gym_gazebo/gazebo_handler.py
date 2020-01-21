#!/usr/bin/env python3
"""
Defines the GazeboHandler class.
"""

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from simulation_handler import SimulationHandler


GAZEBO_SERVICES_MAP = {
    'reset': ['/gazebo/reset_world', Empty],
    'pause': ['/gazebo/pause_physics', Empty],
    'unpause': ['/gazebo/unpause_physics', Empty],
    'set_physics': ['/gazebo/set_physics_properties', SetPhysicsProperties]
}

ODE_PHYSICS_DEFAULT = ODEPhysics()
ODE_PHYSICS_DEFAULT.auto_disable_bodies = False
ODE_PHYSICS_DEFAULT.sor_pgs_precon_iters = 0
ODE_PHYSICS_DEFAULT.sor_pgs_iters = 50
ODE_PHYSICS_DEFAULT.sor_pgs_w = 1.3
ODE_PHYSICS_DEFAULT.sor_pgs_rms_error_tol = 0.0
ODE_PHYSICS_DEFAULT.contact_surface_layer = 0.001
ODE_PHYSICS_DEFAULT.contact_max_correcting_vel = 0.0
ODE_PHYSICS_DEFAULT.cfm = 0.0
ODE_PHYSICS_DEFAULT.erp = 0.2
ODE_PHYSICS_DEFAULT.max_contacts = 20


class GazeboHandler(SimulationHandler):
    """
    The simulation handler for performing pause, unpause, spawn, etc operations
    in gazebo.
    """
    def __init__(self, update_physics_params_at_start=False):
        super(GazeboHandler, self).__init__()
        self.update_physics_params_at_start = update_physics_params_at_start
        self.services = {}

    def setup(self):
        """
        Performs initial simulator setup
        """
        # Get simulation handler services
        for name, _ in GAZEBO_SERVICES_MAP.items():
            sname = GAZEBO_SERVICES_MAP[name][0]
            stype = GAZEBO_SERVICES_MAP[name][1]
            self._check_service_ready(sname)
            self.services[name] = rospy.ServiceProxy(sname, stype)

        super(GazeboHandler, self).setup()

    def reset(self):
        """
        Resets the simulation world
        """
        try:
            self.services['reset']()
        except rospy.ServiceException as exc:
            rospy.logerr(
                'Failed to call reset service with the following error: \
                {}.'.format(exc))

    def pause(self):
        """
        Pauses the simulation world
        """
        try:
            self.services['pause']()
        except rospy.ServiceException as exc:
            rospy.logerr('Failed to call pause service with the following \
            error: {}.'.format(exc))

    def unpause(self):
        """
        Unpauses the simulation world
        """
        try:
            self.services['unpause']()
        except rospy.ServiceException as exc:
            rospy.logerr('Failed to call unpause service with the following \
            error: {}.'.format(exc))

    def initialize_physics_params(
            self,
            time_step=Float64(0.001),
            max_update_rate=Float64(1000.0),
            gravity=Vector3(0.0, 0.0, -9.80655),
            ode_physics=ODE_PHYSICS_DEFAULT):
        """
        Updates physics parameters at startup.
        """
        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = time_step.data
        set_physics_request.max_update_rate = max_update_rate.data
        set_physics_request.gravity = gravity
        set_physics_request.ode_config = ode_physics

        try:
            self.services['set_physics'](set_physics_request)
        except rospy.ServiceException as exc:
            rospy.logerr('Failed to call set_physics service with following \
            error: {}.'.format(exc))

    def _check_service_ready(self, name, timeout=5.0):
        """
        Waits for a service to get ready
        """
        try:
            rospy.wait_for_service(name, timeout)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logerr(
                'Service {} unavailable due to following \
                error: {}'.format(name, exc))
