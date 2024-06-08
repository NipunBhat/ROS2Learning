#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from datetime import datetime

class MyNode(Node):
    def __init__(self) -> None:
        super().__init__("py_test")
        self.__counter = 0
        self.get_logger().info("Hello ROS2")
        time = datetime.now().strftime("%H:%M:%S")
        self.get_logger().info(f"Then time is {time}")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self) -> None:
        self.__counter += 1
        self.get_logger().info(f"Hello {self.__counter}")

def main(args = None):
    rclpy.init(args = args)
    node = Node("py_test")
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
