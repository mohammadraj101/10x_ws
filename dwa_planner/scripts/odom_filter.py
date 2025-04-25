#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdomFilterNode(Node):
    def __init__(self):
        super().__init__('odom_filter_node')
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom_filter', 10)
        
        # Store latest messages
        self.latest_odom = None
        self.latest_scan = None

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.process_and_publish()
    
    def scan_callback(self, msg):
        self.latest_scan = msg
        self.process_and_publish()
    
    def process_and_publish(self):
        if self.latest_odom is None or self.latest_scan is None:
            return  # Wait until both messages are received
        
        # Process the odometry and scan data (basic example: just republish odometry)
        filtered_odom = self.latest_odom  # Modify this based on filtering logic
        
        # Publish filtered odometry
        self.odom_pub.publish(filtered_odom)
        self.get_logger().info("Published filtered odom")


def main(args=None):
    rclpy.init(args=args)
    node = OdomFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
