#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartPhone(Node):
    def __init__(self):
        super().__init__("smart_phone")
        self._subscription = self.create_subscription(String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Started a smart phone subscriber")
    
    def callback_robot_news(self, message : String):
      self.get_logger().info(message.data) # call back function that handles the published data


def main(args = None):
    rclpy.init(args=args)
    node = SmartPhone()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
