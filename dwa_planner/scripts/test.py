#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

class TimerExample(Node):
    def __init__(self):
        super().__init__('timer_example_node')
        self.get_logger().info('Timer example node started')
        
        # Create a timer that calls the timer_callback function every 1 second (1.0 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # This function will be called every 1 second
        self.get_logger().info('Timer callback triggered!')
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = TimerExample()
    rclpy.spin(node)  # Keep the node running and listening for timer callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

