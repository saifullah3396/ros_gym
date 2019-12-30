from airsim_handler import AirsimHandler
from robot_sim_env import RobotSimEnv
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

class RobotAirSimEnv(RobotSimEnv):

    def __init__(self, robot_name_space):
        super(RobotAirSimEnv, self).__init__(robot_name_space, AirsimHandler())
        
    @staticmethod
    def airsim_to_ros_pose(airsim_position, airsim_orientation):
        ros_pose = PoseStamped()
        ros_pose.pose.position.x = airsim_position.x_val
        ros_pose.pose.position.y = airsim_position.y_val
        ros_pose.pose.position.z = airsim_position.z_val
        ros_pose.pose.orientation.w = airsim_orientation.w_val  
        ros_pose.pose.orientation.x = airsim_orientation.x_val
        ros_pose.pose.orientation.y = airsim_orientation.y_val
        ros_pose.pose.orientation.z = airsim_orientation.z_val
        return ros_pose
    
    @staticmethod
    def airsim_to_ros_twist(airsim_lin_vel, airsim_ang_vel):
        ros_twist = TwistStamped()
        ros_twist.twist.linear.x = airsim_lin_vel.x_val
        ros_twist.twist.linear.y = airsim_lin_vel.y_val
        ros_twist.twist.linear.z = airsim_lin_vel.z_val
        ros_twist.twist.angular.x = airsim_ang_vel.x_val
        ros_twist.twist.angular.y = airsim_ang_vel.y_val
        ros_twist.twist.angular.z = airsim_ang_vel.z_val
        return ros_twist
    
    @staticmethod
    def airsim_image_to_numpy(airsim_img_response):
        img1d = np.fromstring(airsim_img_response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(airsim_img_response.height, airsim_img_response.width, 4)
        img_rgb = np.flipud(img_rgb)
        return img_rgb