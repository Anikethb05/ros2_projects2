#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import random
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/gesture_arm/cmd_vel', 10)

        # Subscriber for LIDAR data
        self.create_subscription(LaserScan, '/gesture_arm/scan', self.lidar_callback, qos_profile_sensor_data)

        # Parameters
        self.threshold = 0.5  # meters
        self.base_linear_velocity = 0.5  # faster forward speed
        self.base_angular_velocity = 1.2  # rad/s for turning

        # State tracking
        self.state = 'MOVE_FORWARD'
        self.turn_timer = None

        # This will hold the current turn duration dynamically
        self.turn_duration_sec = 0.0

        self.get_logger().info('Obstacle Avoidance Node is up and running.')

    def lidar_callback(self, msg):
        # Ignore lidar data while turning
        if self.state != 'MOVE_FORWARD':
            return

        ranges = msg.ranges
        ranges_len = len(ranges)

        # Get indices roughly centered in front of robot
        front_indices = list(range(ranges_len//2 - 7, ranges_len//2 + 8))

        # Filter valid distances (ignore zero or inf)
        front_distances = [ranges[i] for i in front_indices if 0 < ranges[i] < float('inf')]

        if not front_distances:
            front = float('inf')  # no obstacle detected
        else:
            front = min(front_distances)

        self.get_logger().info(f"Front distance: {front:.2f} meters")

        if front > self.threshold:
            self.move_forward()
        else:
            # Obstacle detected, turn randomly left or right by 135 to 180 degrees
            direction = random.choice(['left', 'right'])
            # Random angle in radians between 135° (3*pi/4) and 180° (pi)
            turn_angle = random.uniform(3*math.pi/4, math.pi)
            # Calculate turn duration based on angular velocity
            self.turn_duration_sec = turn_angle / self.base_angular_velocity

            self.state = 'TURNING'
            self.get_logger().info(f"Obstacle detected! Turning {direction} for {self.turn_duration_sec:.2f} seconds.")
            self.start_turn(direction)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.base_linear_velocity
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Moving forward")

    def start_turn(self, direction):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.base_angular_velocity if direction == 'left' else -self.base_angular_velocity
        self.cmd_vel_pub.publish(twist)

        # Cancel any existing timer and start new one for turn duration
        if self.turn_timer:
            self.turn_timer.cancel()
        self.turn_timer = self.create_timer(self.turn_duration_sec, self.stop_turn)

    def stop_turn(self):
        if self.turn_timer:
            self.turn_timer.cancel()
            self.turn_timer = None

        self.get_logger().info("Turn complete. Resuming forward motion.")
        self.state = 'MOVE_FORWARD'
        self.move_forward()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
