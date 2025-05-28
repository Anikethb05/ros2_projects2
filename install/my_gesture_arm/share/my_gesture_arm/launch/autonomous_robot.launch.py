#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class DiffDrive(Node):
    def _init_(self):
        super()._init_('obstacle_avoider')
        self.cmd_pub = self.create_publisher(Twist, '/gesture_arm/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        front_ranges = msg.ranges[165:195]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        min_distance = min(valid_ranges) if valid_ranges else float('inf')

        cmd = Twist()
        cmd.linear.x = 0.2  

        if min_distance < 1.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 2.0
            self.get_logger().info(f"Obstacle at {min_distance:.2f} m! Turning...")
        else:
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info('Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()      
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()