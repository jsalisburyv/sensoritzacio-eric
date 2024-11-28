#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import termios
import tty

class InteractiveKeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        # Create publishers for custom topics
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Robot parameters
        self.wheel_separation = 0.122
        self.wheel_base = 0.156
        self.wheel_radius = 0.026
        self.wheel_steering_y_offset = 0.03
        self.steering_track = self.wheel_separation - 2 * self.wheel_steering_y_offset

        # Control variables
        self.linear_speed = 20 # Speed for forward/backward motion
        self.angular_speed = 10  # Speed for turning
        self.vel_msg = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self.mode_selection = 4  # Default: None (1: opposite phase, 2: in-phase, 3: pivot turn, 4: stop)

        # Current control values
        self.positions = [0.0, 0.0, 0.0, 0.0]  # Position commands
        self.velocities = [0.0, 0.0, 0.0, 0.0]  # Velocity commands

        # Display instructions
        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("Interactive Keyboard Controller Initialized!")
        self.get_logger().info("Use W/A/S/D to control the robot, SPACE to stop, and Q to quit.")
        self.get_logger().info("[W]: Forward  [S]: Backward  [A]: Turn Left  [D]: Turn Right")
        self.get_logger().info("[SPACE]: Stop  [Q]: Quit")

    def compute_wheel_commands(self):
        """Compute wheel velocities and positions based on the selected mode and velocities."""
        if self.mode_selection == 1:  # Opposite phase
            vel_steering_offset = self.vel_msg['angular_z'] * self.wheel_steering_y_offset
            sign = math.copysign(1, self.vel_msg['linear_x'])

            self.velocities[0] = sign * math.hypot(self.vel_msg['linear_x'] - self.vel_msg['angular_z'] * self.steering_track / 2,
                                                   self.vel_msg['angular_z'] * self.wheel_base / 2) - vel_steering_offset
            self.velocities[1] = sign * math.hypot(self.vel_msg['linear_x'] + self.vel_msg['angular_z'] * self.steering_track / 2,
                                                   self.vel_msg['angular_z'] * self.wheel_base / 2) + vel_steering_offset
            self.velocities[2] = self.velocities[0]
            self.velocities[3] = self.velocities[1]

            a0 = 2 * self.vel_msg['linear_x'] + self.vel_msg['angular_z'] * self.steering_track
            a1 = 2 * self.vel_msg['linear_x'] - self.vel_msg['angular_z'] * self.steering_track

            self.positions[0] = math.atan(self.vel_msg['angular_z'] * self.wheel_base / a0) if a0 != 0 else 0.0
            self.positions[1] = math.atan(self.vel_msg['angular_z'] * self.wheel_base / a1) if a1 != 0 else 0.0
            self.positions[2] = -self.positions[0]
            self.positions[3] = -self.positions[1]

        elif self.mode_selection == 2:  # In-phase
            V = math.hypot(self.vel_msg['linear_x'], self.vel_msg['linear_y'])
            sign = math.copysign(1, self.vel_msg['linear_x'])

            ang = self.vel_msg['linear_y'] / self.vel_msg['linear_x'] if self.vel_msg['linear_x'] != 0 else 0.0
            self.positions = [math.atan(ang)] * 4
            self.velocities = [sign * V] * 4

        elif self.mode_selection == 3:  # Pivot turn
            self.positions[0] = -math.atan(self.wheel_base / self.steering_track)
            self.positions[1] = math.atan(self.wheel_base / self.steering_track)
            self.positions[2] = self.positions[1]
            self.positions[3] = self.positions[0]

            self.velocities[0] = -self.vel_msg['angular_z']
            self.velocities[1] = self.vel_msg['angular_z']
            self.velocities[2] = self.velocities[0]
            self.velocities[3] = self.velocities[1]

        else:  # Stop
            self.positions = [0.0] * 4
            self.velocities = [0.0] * 4

    def publish_commands(self):
        """Publishes the current position and velocity commands."""
        pos_msg = Float64MultiArray(data=self.positions)
        vel_msg = Float64MultiArray(data=self.velocities)

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

    def get_key(self):
        """Gets a single key press from the user."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Main loop to listen for key presses and control the robot."""
        try:
            while True:
                key = self.get_key().lower()

                if key == 'w':  # Forward
                    self.vel_msg['linear_x'] = self.linear_speed
                    self.mode_selection = 2

                elif key == 's':  # Backward
                    self.vel_msg['linear_x'] = -self.linear_speed
                    self.mode_selection = 2

                elif key == 'a':  # Turn Left
                    self.vel_msg['angular_z'] = self.angular_speed
                    self.mode_selection = 3

                elif key == 'd':  # Turn Right
                    self.vel_msg['angular_z'] = -self.angular_speed
                    self.mode_selection = 3

                elif key == ' ':  # Stop
                    self.vel_msg = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
                    self.mode_selection = 4

                elif key == 'q':  # Quit
                    self.get_logger().info("Quitting. Stopping the robot.")
                    self.vel_msg = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
                    self.mode_selection = 4
                    self.compute_wheel_commands()
                    self.publish_commands()
                    break

                else:
                    self.get_logger().info("Invalid key! Use W/A/S/D/SPACE/Q.")

                # Compute wheel commands and publish
                self.compute_wheel_commands()
                self.publish_commands()

        except KeyboardInterrupt:
            self.get_logger().info("\nExiting controller.")
            self.vel_msg = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
            self.mode_selection = 4
            self.compute_wheel_commands()
            self.publish_commands()

def main(args=None):
    rclpy.init(args=args)
    controller = InteractiveKeyboardController()

    try:
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
