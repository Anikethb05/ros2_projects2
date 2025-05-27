#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import time
from enum import Enum
import threading
from tf_transformations import euler_from_quaternion

class State(Enum):
    NAVIGATE_TO_PICKUP = 1
    PICK_OBJECT = 2
    NAVIGATE_TO_CENTER = 3
    PLACE_OBJECT = 4
    STOP = 5

class AutoNavigator(TeleopController):
    def __init__(self):
        super().__init__()
        # Publishers and subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gesture_arm/odom',
            self.odom_callback,
            10
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/gesture_arm/goal_pose', 10)
        
        # Parameters
        self.current_state = State.NAVIGATE_TO_PICKUP
        self.current_pose = None
        self.pickup_point = (2.0, 2.0)  # Corner of the environment (x, y)
        self.center_point = (0.0, 0.0)  # Center of the environment (x, y)
        self.goal_tolerance = 0.2  # meters
        self.obstacle_threshold = 0.5  # meters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        
        # Environment boundaries (assuming a 5x5 meter area)
        self.bounds = {'x_min': -2.5, 'x_max': 2.5, 'y_min': -2.5, 'y_max': 2.5}
        
        self.get_logger().info('Autonomous Navigator Initialized')
        self.get_logger().info(f'Initial State: {self.current_state}')
        self.get_logger().info(f'Pickup Point: {self.pickup_point}, Center Point: {self.center_point}')

    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose = msg.pose.pose
        orientation_q = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_pose_yaw = yaw

    def get_distance_to_goal(self, goal_point):
        """Calculate Euclidean distance to goal"""
        if self.current_pose is None:
            return float('inf')
        dx = goal_point[0] - self.current_pose.position.x
        dy = goal_point[1] - self.current_pose.position.y
        return np.sqrt(dx**2 + dy**2)

    def get_angle_to_goal(self, goal_point):
        """Calculate angle to goal"""
        if self.current_pose is None:
            return 0.0
        dx = goal_point[0] - self.current_pose.position.x
        dy = goal_point[1] - self.current_pose.position.y
        target_angle = np.arctan2(dy, dx)
        angle_diff = target_angle - self.current_pose_yaw
        # Normalize angle to [-pi, pi]
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
        return angle_diff

    def check_boundaries(self, goal_point):
        """Ensure goal point is within boundaries"""
        x = max(self.bounds['x_min'], min(self.bounds['x_max'], goal_point[0]))
        y = max(self.bounds['y_min'], min(self.bounds['y_max'], goal_point[1]))
        return (x, y)

    def avoid_obstacles(self):
        """Check LIDAR data and adjust velocity to avoid obstacles"""
        if self.obstacle_distance < self.obstacle_threshold:
            # Stop linear motion and turn away
            return 0.0, np.sign(np.random.randn()) * self.angular_speed
        return None

    def navigate_to(self, goal_point):
        """Navigate to a specified goal point"""
        goal_point = self.check_boundaries(goal_point)
        distance = self.get_distance_to_goal(goal_point)
        if distance < self.goal_tolerance:
            return True, 0.0, 0.0  # Reached goal
        
        # Check for obstacles
        avoidance = self.avoid_obstacles()
        if avoidance:
            return False, avoidance[0], avoidance[1]
        
        # Calculate control commands
        angle_diff = self.get_angle_to_goal(goal_point)
        linear_vel = self.linear_speed if distance > self.goal_tolerance else 0.0
        angular_vel = self.angular_speed * np.clip(angle_diff, -1.0, 1.0)
        
        return False, linear_vel, angular_vel

    def run(self):
        try:
            rate = self.create_rate(10)  # 10 Hz
            while rclpy.ok():
                twist = Twist()
                
                if self.current_pose is None:
                    self.get_logger().warn('Waiting for odometry data...')
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                
                if self.current_state == State.NAVIGATE_TO_PICKUP:
                    reached, linear, angular = self.navigate_to(self.pickup_point)
                    if reached:
                        self.get_logger().info('Reached pickup point')
                        self.current_state = State.PICK_OBJECT
                    twist.linear.x = linear
                    twist.angular.z = angular
                
                elif self.current_state == State.PICK_OBJECT:
                    success = self.pick_object()
                    if success:
                        self.get_logger().info('Object picked, navigating to center')
                        self.current_state = State.NAVIGATE_TO_CENTER
                    else:
                        self.get_logger().warn('Pick failed, retrying...')
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                elif self.current_state == State.NAVIGATE_TO_CENTER:
                    reached, linear, angular = self.navigate_to(self.center_point)
                    if reached:
                        self.get_logger().info('Reached center')
                        self.current_state = State.PLACE_OBJECT
                    twist.linear.x = linear
                    twist.angular.z = angular
                
                elif self.current_state == State.PLACE_OBJECT:
                    success = self.place_object()
                    if success:
                        self.get_logger().info('Object placed, stopping')
                        self.current_state = State.STOP
                    else:
                        self.get_logger().warn('Place failed, retrying...')
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                elif self.current_state == State.STOP:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info('Task completed, robot stopped')
                
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.1)
                rate.sleep()
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        
        finally:
            # Stop robot and return arm to home
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.move_arm_to_position(self.home_position)
            if self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    navigator = AutoNavigator()
    navigator.run()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()