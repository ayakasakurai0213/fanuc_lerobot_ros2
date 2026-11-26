import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TwistKeyboard(Node):
    def __init__(self):
        super().__init__("twist_keyboard_subscriber")
        
        self.subscription = self.create_subscription(
            TwistStamped,
            "/twist_keyboard/cmd_vel",
            self.callback,
            10,
        )
        self.twist_data = None
    
    def callback(self, msg):
        self.twist_data = msg.twist
            