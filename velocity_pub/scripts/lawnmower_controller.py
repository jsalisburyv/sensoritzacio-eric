#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time


class OdometryBasedController(Node):
    def __init__(self):
        super().__init__('odometry_based_controller')

        # Robot parameters
        self.wheel_separation = 0.122
        self.wheel_base = 0.156
        self.wheel_radius = 0.026
        self.wheel_steering_y_offset = 0.03
        self.steering_track = self.wheel_separation - 2 * self.wheel_steering_y_offset
        self.linear_speed = 50  # Forward speed in meters/second
        self.angular_speed = 25  # Angular speed in radians/second

        # Robot state
        self.current_position = 0.0  # Distance traveled (meters)
        self.current_orientation = 0.0  # Orientation in radians
        self.prev_left_wheel_pos = None
        self.prev_right_wheel_pos = None

        # Publishers
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Subscribers
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        self.positions = [0.0, 0.0, 0.0, 0.0]  # Position commands
        self.velocities = [0.0, 0.0, 0.0, 0.0]  # Velocity commands

    def joint_states_callback(self, msg):
        # Extract wheel joint positions
        joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}

        # Calculate average left and right wheel positions
        left_wheel_pos = (joint_positions['fl_wheel_joint'] + joint_positions['rl_wheel_joint']) / 2.0
        right_wheel_pos = (joint_positions['fr_wheel_joint'] + joint_positions['rr_wheel_joint']) / 2.0

        # Compute the change in wheel positions
        if self.prev_left_wheel_pos is not None and self.prev_right_wheel_pos is not None:
            delta_left = left_wheel_pos - self.prev_left_wheel_pos
            delta_right = right_wheel_pos - self.prev_right_wheel_pos

            # Calculate the distance traveled by the robot
            delta_distance = self.wheel_radius * (delta_left + delta_right) / 2.0
            self.current_position += delta_distance

            # Calculate the orientation change
            delta_orientation = self.wheel_radius * (delta_right - delta_left) / self.wheel_separation
            self.current_orientation += delta_orientation
            self.current_orientation = math.fmod(self.current_orientation, 2 * math.pi)  # Normalize angle

        # Update previous wheel positions
        self.prev_left_wheel_pos = left_wheel_pos
        self.prev_right_wheel_pos = right_wheel_pos

    def move_forward(self, distance):
        """
        Moves the robot forward by the specified distance.
        :param distance: The distance to move forward in meters.
        """
        self.get_logger().info(f"Moving forward by {distance} meters.")
        self.current_position = 0.0  # Reset the position tracker

        # Set the velocities for forward motion
        self.velocities = [self.linear_speed] * 4
        self.positions = [0.0] * 4

        while rclpy.ok():
            # Check if the target distance is reached
            if self.current_position >= distance:
                self.get_logger().info("Target distance reached.")
                break

            # Publish velocity commands
            self.publish_commands()
            rclpy.spin_once(self)

        # Stop the robot after reaching the target distance
        self.stop()

    def turn(self, angle):
        """
        Turns the robot by the specified angle in degrees.
        :param angle: The angle to turn in degrees (positive for left, negative for right).
        """
        target_angle = math.radians(angle)  # Convert target angle to radians
        initial_orientation = self.current_orientation
        self.get_logger().info(f"Turning by {angle} degrees (target angle: {target_angle:.2f} rad).")

        # Set steering positions for a pivot turn
        steering_angle = math.atan(self.wheel_base / self.steering_track)
        if angle > 0:  # Turn left
            self.positions = [-steering_angle, steering_angle, steering_angle, -steering_angle]
        else:  # Turn right
            self.positions = [steering_angle, -steering_angle, -steering_angle, steering_angle]

        # Set angular velocities for turning
        if angle > 0:  # Turn left
            self.velocities = [-self.angular_speed, self.angular_speed, -self.angular_speed, self.angular_speed]
        else:  # Turn right
            self.velocities = [self.angular_speed, -self.angular_speed, self.angular_speed, -self.angular_speed]

        while rclpy.ok():
            # Check if the target orientation is reached
            angle_turned = self.current_orientation - initial_orientation
            if abs(angle_turned) >= abs(target_angle):
                self.get_logger().info("Target angle reached.")
                break

            # Publish velocity and position commands
            self.publish_commands()
            rclpy.spin_once(self)

        # Stop the robot after reaching the target angle
        self.stop()



    def stop(self):
        """Stops the robot."""
        self.get_logger().info("Stopping the robot.")
        self.velocities = [0.0] * 4
        self.positions = [0.0] * 4
        self.publish_commands()
        time.sleep(0.5)  # Allow time for the robot to stop completely

    def publish_commands(self):
        """Publishes the current position and velocity commands."""
        pos_msg = Float64MultiArray(data=self.positions)
        vel_msg = Float64MultiArray(data=self.velocities)

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = OdometryBasedController()
    x_max = 3
    y_max = 3
    current_row = 0
    try:
        while current_row < y_max:
            turn_angle = 90 if current_row % 2 == 0 else -90
            controller.move_forward(x_max)
            controller.turn(turn_angle)
            controller.move_forward(1)
            controller.turn(turn_angle)
            current_row += 1
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
