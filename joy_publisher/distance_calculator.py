#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from exomy_msgs.msg import MotorCommands
import math

class MotorCommandsListener(Node):
    def __init__(self):
        # Initialize the class variables
        super().__init__('motor_commands_listener')
        self.driving = False
        self.angular_speed = 5.01775  # rad/s
        self.wheel_radius = 0.047  # m
        self.t = None  # Initialize time variable
        self.total_distance = 0  # Total distance traveled

        # Subscribe to the /motor_commands topic
        self.subscriber = self.create_subscription(
            MotorCommands,
            '/exomy/motor_commands_with_timestamp',
            self.callback,
            10)

        # Timer to periodically check the distance calculation
        self.timer = self.create_timer(0.1, self.integrate)  # 0.1 seconds interval

    def callback(self, msg):
        # Extract the timestamp from the header
        timestamp = msg.header.stamp
        motor_speeds = msg.motor_speeds
        motor_angles = msg.motor_angles

        # Update driving status based on motor speeds
        if motor_speeds[0] != 0:
            self.driving = True
        else:
            self.driving = False
            if self.t is not None:
                self.get_logger().info(f"Stopped driving at {timestamp.sec}.{timestamp.nanosec}s")
                self.t = None  # Reset time when stopping

        # Update the timestamp when receiving a message
        self.t = timestamp.sec * 1e9 + timestamp.nanosec  # Store time in nanoseconds

    def integrate(self):
        # Calculate the distance if the robot is driving
        if self.driving and self.t is not None:
            t2 = self.get_clock().now().nanoseconds  # Get the current time in nanoseconds
            time_diff = t2 - self.t
            self.t = t2  # Update the timestamp for the next iteration
            # Convert time difference to seconds
            time_diff_sec = time_diff / 1e9
            distance = 0.236 * time_diff_sec
            self.total_distance += distance
            self.get_logger().info(f"Distance traveled so far: {self.total_distance} meters")

    def spin(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    listener = MotorCommandsListener()
    listener.spin()
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
