#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("robot_news_station") #creating a node
        
        self.publisher_name = "Nipun"
        self.create_timer(0.5, self.publish_news)

        self._publisher = self.create_publisher(String, "robot_news", 10)
        self.get_logger().info("Robot News Station Publisher has been started")

    def publish_news(self) -> None:
        msg = String()
        msg.data = f"Hello this is {self.publisher_name} from ROS2 Publisher Node"
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()