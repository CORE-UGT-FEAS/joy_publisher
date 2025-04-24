#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import sys
import tty
import termios
import select

class JoyController(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__('joy_controller')
        self.get_logger().info('Joy Controller Node Started')

        # Create publisher for joy topic
        self.publisher_ = self.create_publisher(Joy, '/exomy/joy', 10)

        # Initialize joystick axes and buttons state
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example joystick axes data
        self.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # Example joystick buttons data

    def change_mode(self, mode):
        if mode == '1':
            self.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        elif mode == '2':
            self.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        elif mode == '3':
            self.buttons = [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # NOT WORKING idk what number crabbing is have to figure out listening to joystick in website when publishing in /joy

        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def go_front(self):
        self.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def turn_right(self):
        self.axes = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def turn_left(self):
        self.axes = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def go_back(self):
        self.axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_message()

    def stop(self):
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Stop
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.publish_message()

    def publish_message(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = self.axes
        msg.buttons = self.buttons
        self.publisher_.publish(msg)

    def start(self):
        self.get_logger().info("Listening for keyboard input... Press 'q' to quit.")

        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())  # Set terminal to cbreak mode

            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:  # Non-blocking input
                    key = sys.stdin.read(1)  # Read a single character

                    if key == 'w':
                        self.change_mode('1')
                        self.go_front()
                    elif key == 'a':
                        self.change_mode('2')
                        self.turn_left()
                    elif key == 'd':
                        self.change_mode('2')
                        self.turn_right()
                    elif key == 's':
                        self.change_mode('1')
                        self.stop()
                    elif key == 'z':
                        self.change_mode('1')
                        self.go_back()
                    elif key == 'q':
                        self.change_mode('1')
                        self.stop()
                        self.get_logger().info("Quitting controller")
                        break  # Exit loop

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)



def main(args=None):
    rclpy.init(args=args)
    joy_controller = JoyController()
    joy_controller.start()  # Start the controll sequence

    joy_controller.destroy_node()
    rclpy.shutdown()
    exit


if __name__ == '__main__':
    main()