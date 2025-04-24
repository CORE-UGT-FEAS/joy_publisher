#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

class JoyPublisher(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__('joy_publisher')
        self.get_logger().info('Joy Publisher Node Started')

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

    def square_up(self, square_size, turn_time, refresh_time):
        for i in range(4):
            # Change mode to Ackermann
            self.change_mode('1')
            time.sleep(refresh_time)  # Sleep for the specified refresh time
            self.go_front()
            time.sleep(refresh_time)

            # Change mode to Spot Turning
            self.change_mode('2')
            time.sleep(refresh_time)
            # turn right for 1 second
            self.turn_right()
            time.sleep(turn_time)

    def line_up(self, line_size, turn_time, refresh_time):
        for i in range(1):
            # Change mode to Ackermann
            self.change_mode('1')
            self.go_front()

            time.sleep(line_size)

    def spin_out(self, turn_time, direction, refresh_time):
        # Change mode to Spot Turning
        self.change_mode('2')
        # turn right for 1 second
        if direction != 1:
            self.turn_left()
        else:
            self.turn_right()

        self.get_logger().info('sleeping')
        time.sleep(turn_time)  # Sleep for the specified refresh time
        self.get_logger().info('waking up')

    def start(self):
        # Start with Ackermann mode
        self.change_mode('1')
        self.get_logger().info('sleeping')
        time.sleep(0.5)  # Sleep for 0.5 seconds instead of rate.sleep()
        self.get_logger().info('waking up')

        # Sequence of movements
        while rclpy.ok():
            self.get_logger().info('Sequence started')
            # Perform sequence
            time.sleep(2)

            self.line_up(1.1/(1.11/(20)), 2.2, 0.5)
            self.stop()
            time.sleep(1)

            self.spin_out(18.5/4, 1, 0.5) #18.5s 180° for right turn
            self.stop()
            time.sleep(1)

            self.line_up(1.9/(1.11/(20)), 2.2, 0.5)
            self.stop()
            time.sleep(1)

            self.spin_out(20/4, -1, 0.5) #18.5s 180° for right turn
            self.stop()
            time.sleep(1)

            self.line_up(0.6/(1.11/(20)), 2.2, 0.5)
            self.stop()
            time.sleep(1)

            #self.line_up(20, 2.2, 0.5)
            #self.stop()
            #time.sleep(1)
#
            #self.spin_out(18.5/2, 1, 0.5) #18.5s 180° for right turn
            #self.stop()
            #time.sleep(1)
#
            #self.line_up(20, 2.2, 0.5)
            #self.stop()
            #time.sleep(1)
#
            #self.spin_out(18.5/2, 1, 0.5) #19.5s 180° for lrft turn
            #self.stop()
            #time.sleep(1)
#
            #self.line_up(15, 2.2, 0.5)
            #self.stop()
            #time.sleep(1)

            # Stop after the sequence
            self.change_mode('1')
            self.stop()
            break
        self.get_logger().info('Sequence completed')


def main(args=None):
    rclpy.init(args=args)
    joy_publisher = JoyPublisher()
    joy_publisher.start()  # Start the movement sequence
    rclpy.spin(joy_publisher)  # Keep the node running

    joy_publisher.destroy_node()
    rclpy.shutdown()
    exit


if __name__ == '__main__':
    main()