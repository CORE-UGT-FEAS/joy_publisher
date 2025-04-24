#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped  # Assuming imu_link publishes PoseStamped
import time
import math
import tf2_ros
from scipy.spatial.transform import Rotation as R

class JoyPublisher(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__('joy_publisher')
        self.get_logger().info('Joy Publisher Node Started')

        # Create publisher for joy topic
        self.publisher_ = self.create_publisher(Joy, '/exomy/joy', 10)

        # Subscribe to imu_link for position & rotation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to update position
        self.imu_connected = False
        self.timer = self.create_timer(0.1, self.update_position)  # 0.1s interval
        self.timer2 = self.create_timer(0.1, self.move_distance)  # 0.1s interval

        # Initialize values
        self.current_yaw = 0.0
        self.current_position = [0.0, 0.0]  # [x, y]

        # Initialize joystick axes and buttons state
        self.axes = [0.0] * 6
        self.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.goal = [0.0,1.0]  #y,x
        self.change_mode('1')  # Ackermann mode

    def update_position(self):
        self.get_logger().warn("Updating position...")
        try:
            transform = self.tf_buffer.lookup_transform('world', 'imu_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            self.current_position = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
            # Create a Rotation object from the quaternion
            r = R.from_quat([qx, qy, qz, qw])

            # Extract the yaw (Euler angles in 'xyz' order, where z is yaw)
            _, _, yaw = r.as_euler('xyz', degrees=False)
            self.current_yaw = -yaw
            self.get_logger().info(f"Position: {self.current_position}°")
            self.get_logger().info(f"tf: {transform}°")
            self.imu_connected = True


        except tf2_ros.LookupException:
            self.get_logger().warn("Transform not found, skipping transformation")
            self.imu_connected = False

    ### --- CALLBACKS FOR SENSORS --- ###
    def update_pose(self, msg):
        """ Updates position and yaw from imu_link. """
        self.current_position = [msg.pose.position.x, msg.pose.position.y]
        self.current_yaw = self.quaternion_to_yaw(msg.pose.orientation)

    def quaternion_to_yaw(self, q):
        """ Converts quaternion to yaw (rotation around Z-axis). """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp) * (180 / math.pi)  # Convert to degrees

    ### --- MOVEMENT FUNCTIONS --- ###
    def change_mode(self, mode):
        """ Changes control mode (1: Ackermann, 2: Spot Turn). """
        self.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] if mode == '1' else [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.axes = [0.0] * 6
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

    def stop(self):
        self.axes = [0.0] * 6
        self.buttons = [0] * 12
        self.publish_message()

    ### --- HARD-CODED MOVEMENT SEQUENCES --- ###
    def turn_to_angle(self, target_yaw):
        """ Turns robot to the given target angle. """
        self.get_logger().info(f"Turning to {target_yaw}°")
        self.change_mode('2')  # Switch to spot turning

        while abs(self.current_yaw - target_yaw) > 2.0:  # Allow small error
            self.turn_right() if self.current_yaw < target_yaw else self.turn_left()
            time.sleep(0.1)

        self.stop()
        self.get_logger().info("Turn completed")

    def move_distance(self):
        """ Moves robot forward a specified distance. """
        if self.imu_connected:
            start_position = self.current_position[:]
            self.get_logger().info(f"Moving forward to {self.goal}")

            if self.current_position[1] < self.goal[1]:
                self.go_front()
                self.get_logger().info(f"Curernt x: {self.current_position[1]} to {self.goal[1]}")
            else:
                self.stop()
            self.get_logger().info("Move completed")

    def get_distance_traveled(self, start_position):
        """ Computes Euclidean distance from start position. """
        return math.sqrt((self.current_position[0] - start_position[0])**2 +
                         (self.current_position[1] - start_position[1])**2)

    ### --- RUNNING A SEQUENCE --- ###
    def start(self):
        """ Hardcoded movement sequence using angles & distances. """
        self.get_logger().info("Starting movement sequence...")

        self.move_distance(1.5)  # Move 2 meters forward
        time.sleep(1)

        self.turn_to_angle(90)  # Turn to 90°
        time.sleep(1)

        self.move_distance(1.)  # Move 1.5 meters forward
        time.sleep(1)

        self.turn_to_angle(180)  # Turn to 180°
        time.sleep(1)

        self.move_distance(1.)  # Move 3 meters forward
        time.sleep(1)

        self.turn_to_angle(270)  # Turn to 270°
        time.sleep(1)

        self.move_distance(1.5)  # Move 2.5 meters forward
        time.sleep(1)

        self.turn_to_angle(0)  # Face forward again
        time.sleep(1)

        self.stop()
        self.get_logger().info("Movement sequence completed")

    def publish_message(self):
        """ Publishes the joystick command. """
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = self.axes
        msg.buttons = self.buttons
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joy_publisher = JoyPublisher()
    rclpy.spin(joy_publisher)  # Keep the node running
    joy_publisher.start()  # Start the movement sequence



    joy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()