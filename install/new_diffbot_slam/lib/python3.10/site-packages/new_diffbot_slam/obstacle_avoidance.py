#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.cmd_pub = self.create_publisher(Twist, '/diffbot/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('Obstacle Avoidance Node is up and running.')

    def scan_callback(self, msg):
        front_ranges = msg.ranges[165:195]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        min_distance = min(valid_ranges) if valid_ranges else float('inf')

        cmd = Twist()
        cmd.linear.x = 0.3

        if min_distance < 1.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5
            self.get_logger().info(f'Obstacle at {min_distance:.2f} m! Turning...')
        else:
            cmd.angular.z = 0.0
            import random
            if random.random() < 0.05:
                cmd.angular.z = random.uniform(-0.5, 0.5)

        if rclpy.ok():
            try:
                self.cmd_pub.publish(cmd)
            except Exception as e:
                self.get_logger().error(f'Failed to publish cmd_vel: {e}')

    def stop_robot(self):
        if rclpy.ok():
            stop_cmd = Twist()
            try:
                self.cmd_pub.publish(stop_cmd)
                self.get_logger().info('Robot stopped.')
            except Exception as e:
                self.get_logger().error(f'Failed to publish stop command: {e}')

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        node.get_logger().info('Node shutdown complete')

if __name__ == '__main__':
    main()