#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest, DeleteModel
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

from simulation_handler import SimulationHandler

gazebo_services_dict = {
    'reset' : ['/gazebo/reset_world', Empty],
    'pause' : ['/gazebo/pause_physics', Empty],
    'unpause' : ['/gazebo/unpause_physics', Empty],
    'set_physics' : ['/gazebo/set_physics_properties', SetPhysicsProperties]
}

ode_physics_default = ODEPhysics()
ode_physics_default.auto_disable_bodies = False
ode_physics_default.sor_pgs_precon_iters = 0
ode_physics_default.sor_pgs_iters = 50
ode_physics_default.sor_pgs_w = 1.3
ode_physics_default.sor_pgs_rms_error_tol = 0.0
ode_physics_default.contact_surface_layer = 0.001
ode_physics_default.contact_max_correcting_vel = 0.0
ode_physics_default.cfm = 0.0
ode_physics_default.erp = 0.2
ode_physics_default.max_contacts = 20

class GazeboHandler(SimulationHandler):
    
    def __init__(self, update_physics_params_at_start=False):
        super(GazeboHandler, self).__init__()
        self.update_physics_params_at_start = update_physics_params_at_start

    def setup(self):
        """
        Performs initial simulation setup
        """
        
        self.services = {}
        # Get simulation handler services
        for name, service in gazebo_services_dict.items():
            sname = gazebo_services_dict[name][0]
            stype = gazebo_services_dict[name][1]
            self._check_service_ready(sname)
            self.services[name] = rospy.ServiceProxy(sname, stype) 
            
        print("calling simulation handler setup()")
        super(GazeboHandler, self).setup()

    def reset(self):
        try:
            self.services['reset']()
        except rospy.ServiceException as e:
            rospy.logerr ("Failed to call reset service.")

    def pause(self):
        try:
            self.services['pause']()
        except rospy.ServiceException as e:
            rospy.logerr('Failed to call pause service.')
        
    def unpause(self):
        try:
            self.services['unpause']()
        except rospy.ServiceException as e:
            rospy.logerr('Failed to call unpause service.')

    def initialize_physics_params(
        self,
        time_step=Float64(0.001), 
        max_update_rate=Float64(1000.0),
        gravity=Vector3(0.0, 0.0, -9.80655),
        ode_physics=ode_physics_default):
        
        """
        Updates physics parameters at startup.
        """

        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = time_step.data
        set_physics_request.max_update_rate = max_update_rate.data
        set_physics_request.gravity = gravity
        set_physics_request.ode_config = ode_physics

        try:
            result = self.services['set_physics'](set_physics_request)
        except rospy.ServiceException as e:
            rospy.logerr('Failed to call set_physics service.')

    def _check_service_ready(self, name, timeout=5.0):
        """
        Waits for a service to get ready
        """
        try:
            rospy.wait_for_service(name, timeout)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service %s unavailable.", name)