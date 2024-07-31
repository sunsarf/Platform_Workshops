#!usr/bin/env python3

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)  # Create a publisher
        self.timer = self.create_timer(1.0, self.run) #execute run() every 1 second
        self.k = 0
        
    def run(self):
        msg = String()
        msg.data = f'Hello, World!: {self.k}'
        self.publisher.publish(msg)  # Publish the message
        # self.get_logger().info(f'Hello, World!: {self.k}') 
        self.k += 1
        
def main(args=None):
    rclpy.init(args=args) #initialize ROS2 communication
    node = TalkerNode()
    rclpy.spin(node)  #keep the node running until stopped
    rclpy.shutdown()  #shutdown ROS2 communication
    
if __name__ == '__main__':
    main()