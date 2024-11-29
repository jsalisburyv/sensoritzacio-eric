#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import json
import numpy as np


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Publishers for controlling the robot
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Robot parameters
        self.wheel_separation = 0.122
        self.wheel_base = 0.156
        self.wheel_radius = 0.026
        self.wheel_steering_y_offset = 0.03
        self.steering_track = self.wheel_separation - 2 * self.wheel_steering_y_offset

        # Control variables
        self.linear_speed = 20  # Speed for forward/backward motion (same as keyboard controller)
        self.angular_speed = 10  # Speed for turning (same as keyboard controller)
        self.vel_msg = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self.mode_selection = 4  # Default mode (stop)

        # Current control values
        self.positions = [0.0, 0.0, 0.0, 0.0]
        self.velocities = [0.0, 0.0, 0.0, 0.0]

        # Waypoints
        with open('cans.json', 'r') as file:
            data = json.load(file)
        self.models = {can['name']: can['position'][:2] for can in data['cans']}
        self.get_logger().info(f"Loaded models: {self.models}")

        self.start_position = [0.0, 0.0]
        self.waypoints = list(self.models.values())
        self.optimized_path = self.find_optimal_path(self.start_position, self.waypoints)
        self.get_logger().info(f"Optimized path: {self.optimized_path}")

        # State variables
        self.current_index = 0
        self.current_position = np.array([0.0, 0.0])  # Initial position
        self.current_orientation = 0.0  # Initial orientation

        # Timer for navigation
        self.timer = self.create_timer(0.1, self.navigate_to_waypoint)

    def find_optimal_path(self, start, waypoints):
        """Compute the optimal path visiting all waypoints starting from the given position."""
        from itertools import permutations
        min_cost = float('inf')
        best_path = []

        for perm in permutations(waypoints):
            cost = self.compute_path_cost(start, perm)
            if cost < min_cost:
                min_cost = cost
                best_path = perm

        return best_path

    def compute_path_cost(self, start, waypoints):
        """Compute the total distance for visiting all waypoints in the given order."""
        total_cost = 0
        current_pos = start

        for waypoint in waypoints:
            total_cost += self.euclidean_distance(current_pos, waypoint)
            current_pos = waypoint

        return total_cost

    @staticmethod
    def euclidean_distance(a, b):
        """Compute Euclidean distance between two points."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def compute_wheel_commands(self):
        """Compute wheel velocities and positions based on the selected mode and velocities."""
        if self.mode_selection == 2:  # In-phase (default for navigation)
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

    def navigate_to_waypoint(self):
        """Main navigation logic to move the robot towards the next waypoint."""
        if self.current_index >= len(self.optimized_path):
            self.get_logger().info("All waypoints reached!")
            self.mode_selection = 4  # Stop
            self.compute_wheel_commands()
            self.publish_commands()
            return

        # Current target waypoint
        goal = self.optimized_path[self.current_index]
        dx = goal[0] - self.current_position[0]
        dy = goal[1] - self.current_position[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = math.atan2(dy, dx)

        # Angular error
        angle_error = angle_to_goal - self.current_orientation
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Move toward the waypoint
        if distance > 0.2:  # Threshold for reaching the waypoint
            self.vel_msg['linear_x'] = self.linear_speed
            self.vel_msg['angular_z'] = max(-self.angular_speed, min(self.angular_speed, angle_error * 2))
            self.mode_selection = 2  # In-phase mode
        else:
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}: {goal}")
            self.current_index += 1
            self.vel_msg = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
            self.mode_selection = 4  # Stop mode

        self.compute_wheel_commands()
        self.publish_commands()


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()

    try:
        rclpy.spin(waypoint_follower)
    finally:
        waypoint_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
