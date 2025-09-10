#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import sys
import select
import termios
import tty
import numpy as np
import time
import threading

msg = """
Robot Teleop Controller
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold Shift:
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

Arm Control:
p : Pick object (if object is <0.25m in front)
r : Place object (release at current position)
h : Home position (reset arm to neutral)

Speed Control:
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease linear speed by 10%
e/c : increase/decrease angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1.0, 0.0, 0.0, 0.0),    # Forward
    'o': (1.0, 0.0, 0.0, -1.0),   # Forward + turn left
    'j': (0.0, 0.0, 0.0, 1.0),    # Turn right
    'l': (0.0, 0.0, 0.0, -1.0),   # Turn left
    'u': (1.0, 0.0, 0.0, 1.0),    # Forward + turn right
    ',': (-1.0, 0.0, 0.0, 0.0),   # Backward
    '.': (-1.0, 0.0, 0.0, 1.0),   # Backward + turn right
    'm': (-1.0, 0.0, 0.0, -1.0),  # Backward + turn left
    'O': (1.0, -1.0, 0.0, 0.0),   # Forward + strafe left
    'I': (1.0, 0.0, 0.0, 0.0),    # Forward
    'J': (0.0, 1.0, 0.0, 0.0),    # Strafe right
    'L': (0.0, -1.0, 0.0, 0.0),   # Strafe left
    'U': (1.0, 1.0, 0.0, 0.0),    # Forward + strafe right
    '<': (-1.0, 0.0, 0.0, 0.0),   # Backward
    '>': (-1.0, -1.0, 0.0, 0.0),  # Backward + strafe left
    'M': (-1.0, 1.0, 0.0, 0.0),  # Backward + strafe right
    't': (0.0, 0.0, 1.0, 0.0),   # Up
    'b': (0.0, 0.0, -1.0, 0.0),  # Down
}

speedBindings = {
    'q': (1.1, 1.1),  # Increase both speeds
    'z': (0.9, 0.9),  # Decrease both speeds
    'w': (1.1, 1.0),  # Increase linear speed
    'x': (0.9, 1.0),  # Decrease linear speed
    'e': (1.0, 1.1),  # Increase angular speed
    'c': (1.0, 0.9),  # Decrease angular speed
}

class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')
        self.settings = None
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.get_logger().warn('No terminal available. Run in a terminal for keyboard input.')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/gesture_arm/cmd_vel', 10)
        self.shoulder_yaw_pub = self.create_publisher(Float64, '/gesture_arm/shoulder_yaw_joint_controller/command', 10)
        self.shoulder_pitch_pub = self.create_publisher(Float64, '/gesture_arm/shoulder_pitch_joint_controller/command', 10)
        self.elbow_pub = self.create_publisher(Float64, '/gesture_arm/elbow_joint_controller/command', 10)
        self.wrist_roll_pub = self.create_publisher(Float64, '/gesture_arm/wrist_roll_joint_controller/command', 10)
        self.wrist_pitch_pub = self.create_publisher(Float64, '/gesture_arm/wrist_pitch_joint_controller/command', 10)
        self.left_finger_pub = self.create_publisher(Float64, '/gesture_arm/left_finger_joint_controller/command', 10)
        self.right_finger_pub = self.create_publisher(Float64, '/gesture_arm/right_finger_joint_controller/command', 10)

        # Subscriber for LIDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/gesture_arm/scan',
            self.lidar_callback,
            10
        )

        # Parameters
        self.speed = 0.3  # Linear speed (m/s)
        self.turn = 0.5   # Angular speed (rad/s)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0

        # Obstacle detection for picking
        self.obstacle_distance = float('inf')
        self.pick_distance = 0.25  # Distance to allow picking
        self.carrying_object = False

        # Arm positions
        self.home_position = {
            'shoulder_yaw': 0.0,
            'shoulder_pitch': 0.0,
            'elbow': 0.0,
            'wrist_roll': 0.0,
            'wrist_pitch': 0.0,
            'gripper': 0.03  # Open
        }
        self.pick_ready_position = {
            'shoulder_yaw': 0.0,
            'shoulder_pitch': -0.8,
            'elbow': -1.2,
            'wrist_roll': 0.0,
            'wrist_pitch': -0.5,
            'gripper': 0.03  # Open
        }
        self.pick_position = {
            'shoulder_yaw': 0.0,
            'shoulder_pitch': -1.0,
            'elbow': -1.5,
            'wrist_roll': 0.0,
            'wrist_pitch': -0.8,
            'gripper': 0.015  # Closed
        }
        self.carry_position = {
            'shoulder_yaw': 0.0,
            'shoulder_pitch': -0.3,
            'elbow': -0.8,
            'wrist_roll': 0.0,
            'wrist_pitch': 0.0,
            'gripper': 0.015  # Closed
        }
        self.place_position = {
            'shoulder_yaw': 1.57,
            'shoulder_pitch': -0.8,
            'elbow': -1.0,
            'wrist_roll': 0.0,
            'wrist_pitch': -0.3,
            'gripper': 0.03  # Open
        }

        print(msg)
        print(self.vels(self.speed, self.turn))
        print(f"Carrying object: {self.carrying_object}")

    def getKey(self):
        if not self.settings:
            return None
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

    def lidar_callback(self, msg):
        # Check front arc (±30 degrees) for picking
        ranges = np.array(msg.ranges)
        front_angle = np.radians(30)
        num_samples = len(ranges)
        samples_per_degree = num_samples / (2 * np.pi)
        front_samples = int(front_angle * samples_per_degree)
        front_ranges = np.concatenate((ranges[:front_samples], ranges[-front_samples:]))
        valid_ranges = front_ranges[np.isfinite(front_ranges)]
        
        if len(valid_ranges) > 0:
            self.obstacle_distance = np.min(valid_ranges)
        else:
            self.obstacle_distance = float('inf')

    def move_arm_to_position(self, position, duration=2.0):
        """Move arm to specified position with smooth motion"""
        self.get_logger().info(f'Moving arm to position: {position}')
        
        # Publish joint commands
        self.shoulder_yaw_pub.publish(Float64(data=position['shoulder_yaw']))
        self.shoulder_pitch_pub.publish(Float64(data=position['shoulder_pitch']))
        self.elbow_pub.publish(Float64(data=position['elbow']))
        self.wrist_roll_pub.publish(Float64(data=position['wrist_roll']))
        self.wrist_pitch_pub.publish(Float64(data=position['wrist_pitch']))
        
        # Smooth transition
        steps = int(duration / 0.1)
        for _ in range(steps):
            time.sleep(0.1)
        
        # Control gripper
        self.left_finger_pub.publish(Float64(data=position['gripper']))
        self.right_finger_pub.publish(Float64(data=position['gripper']))
        time.sleep(0.5)

    def pick_object(self):
        """Pick up an object if detected <0.25m"""
        if self.carrying_object:
            self.get_logger().warn('Already carrying an object!')
            return False
        
        if self.obstacle_distance > self.pick_distance:
            self.get_logger().warn(f'No object close enough to pick. Distance: {self.obstacle_distance:.2f}m')
            return False
        
        self.get_logger().info('Starting pick sequence...')
        try:
            self.move_arm_to_position(self.pick_ready_position, 2.0)
            self.move_arm_to_position(self.pick_position, 2.5)
            self.move_arm_to_position(self.carry_position, 2.0)
            self.carrying_object = True
            self.get_logger().info('Object picked successfully!')
            return True
        except Exception as e:
            self.get_logger().error(f'Pick failed: {e}')
            return False

    def place_object(self):
        """Place the carried object"""
        if not self.carrying_object:
            self.get_logger().warn('No object to place!')
            return False
        
        self.get_logger().info('Starting place sequence...')
        try:
            self.move_arm_to_position(self.place_position, 2.5)
            self.move_arm_to_position(self.home_position, 2.0)
            self.carrying_object = False
            self.get_logger().info('Object placed successfully!')
            return True
        except Exception as e:
            self.get_logger().error(f'Place failed: {e}')
            return False

    def run(self):
        try:
            while rclpy.ok():
                key = self.getKey()
                if key is None or key == '':
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                if key in moveBindings:
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                
                elif key in speedBindings:
                    self.speed *= speedBindings[key][0]
                    self.turn *= speedBindings[key][1]
                    self.speed = max(0.1, min(self.speed, 0.8))
                    self.turn = max(0.2, min(self.turn, 1.5))
                    print(self.vels(self.speed, self.turn))
                    if self.status == 14:
                        print(msg)
                        print(f"Carrying object: {self.carrying_object}")
                    self.status = (self.status + 1) % 15
                
                elif key == 'p':
                    threading.Thread(target=self.pick_object, daemon=True).start()
                
                elif key == 'r':
                    threading.Thread(target=self.place_object, daemon=True).start()
                
                elif key == 'h':
                    threading.Thread(target=lambda: self.move_arm_to_position(self.home_position), daemon=True).start()
                
                else:
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                    if key == '\x03':  # CTRL-C
                        break

                # Publish movement command (no obstacle checks)
                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.z = self.th * self.turn
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                self.cmd_vel_pub.publish(twist)
                
                rclpy.spin_once(self, timeout_sec=0.1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # Stop robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            # Return arm to home
            #self.move_arm_to_position(self.home_position)
            if self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    controller = TeleopController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
