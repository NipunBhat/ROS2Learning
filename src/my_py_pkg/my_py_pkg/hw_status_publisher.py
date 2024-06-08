#usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HWStatusPublisherNode(Node):
    def __init__(self):
       super().__init__("hw_status_publisher")
       self._hw_status_publisher = self.create_publisher(HardwareStatus, "hardware_status", 10)
       self._timer = self.create_timer(1.0, self.publish_hw_status)
       self.get_logger().info("Hardware Status Message is published")
       
    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_ready = True
        msg.debug_message = "All good"
        self._hw_status_publisher.publish(msg=msg)
        self.get_logger().info("Published Message")
       
    
def main(args = None):
    rclpy.init(args= args)
    node = HWStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
        
