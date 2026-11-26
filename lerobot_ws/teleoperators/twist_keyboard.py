import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory


class TwistKeyboard(Node):
    def __init__(self):
        super().__init__("twist_keyboard_subscriber")
        
        self.arm_sub = self.create_subscription(
            TwistStamped,
            "/twist_keyboard/cmd_vel",
            self.arm_callback,
            10,
        )
        
        self.gripper_sub = self.create_subscription(
            JointTrajectory, 
            "/twist_keyboard/gripper_joint_trajectory", 
            self.gripper_callback, 
            10
        )
        
        self.twist_data = None
        self.grip_data = None
    
    def arm_callback(self, msg):
        self.twist_data = msg.twist
    
    def gripper_callback(self, msg):
        self.grip_data = msg.points[0].positions
            