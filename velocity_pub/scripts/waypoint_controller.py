import time
from model_deleter import ModelDeleter
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.entity_pb2 import Entity
import math
import json


class WaypointFollower:
    def __init__(self):
        self.deleter = ModelDeleter()
        self.node = Node()
        self.publisher = self.node.advertise("/cmd_vel", Twist)
        self.pose_subscriber = self.node.subscribe(msg_type=Pose_V, topic="/joint_states", callback=self.pose_callback)
        self.remove_publisher = self.node.advertise("/world/beach_world/scene/deletion", Entity)

        # Load waypoints from JSON
        with open('cans.json', 'r') as file:
            data = json.load(file)
        self.models = {can['name']: can['position'][:2] for can in data['cans']}
        print(f"Loaded models: {self.models}")

        # Find optimal path
        self.start_position = [0.0, 0.0]  # Starting position of the robot
        self.waypoints = list(self.models.values())  # Extract model positions
        self.optimized_path = self.find_optimal_path(self.start_position, self.waypoints)
        print(f"Optimized path: {self.optimized_path}")

        self.current_index = 0
        self.current_position = self.start_position
        self.current_orientation = None  # Robot's current yaw
        self.timer = time.time()

    def pose_callback(self, msg):
        """
        Updates the robot's current position and checks for collisions.
        """
        for pose in msg.pose:
            if pose.name == "MR-Buggy3":  # Replace with your robot's model name
                # Update position
                self.current_position = [pose.position.x, pose.position.y]
                # Extract yaw from quaternion
                qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                self.current_orientation = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            else:
                # Check collision with each model
                model_name = pose.name
                if model_name in self.models:
                    model_position = self.models[model_name]
                    distance = self.euclidean_distance(self.current_position, model_position)
                    if distance < 0.2:  # Collision threshold
                        print(f"Collision detected with {model_name}. Removing it...")
                        self.deleter.delete_model(model_name)

    def find_optimal_path(self, start, waypoints):
        """
        Compute the optimal path visiting all waypoints starting from the given position.
        Uses a brute-force TSP solution for simplicity.
        """
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
        """
        Compute the total distance for visiting all waypoints in the given order.
        """
        total_cost = 0
        current_pos = start

        for waypoint in waypoints:
            total_cost += self.euclidean_distance(current_pos, waypoint)
            current_pos = waypoint

        return total_cost

    @staticmethod
    def euclidean_distance(a, b):
        """
        Compute Euclidean distance between two points.
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def navigate_to_waypoint(self):
        if self.current_index >= len(self.optimized_path):
            print("All waypoints reached!")
            self.publish_velocity(0.0, 0.0)
            return

        # Ensure we have valid orientation
        if self.current_orientation is None:
            print("Waiting for robot orientation...")
            return

        # Current waypoint
        goal = self.optimized_path[self.current_index]

        # Compute distance and angle to the waypoint
        dx = goal[0] - self.current_position[0]
        dy = goal[1] - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # Compute the angular error
        angle_error = angle_to_goal - self.current_orientation
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]

        # Control logic
        linear_velocity = 0.5 if distance > 0.1 else 0.0
        angular_velocity = max(-1.0, min(1.0, 2.0 * angle_error))  # Cap angular velocity

        # Update waypoint if close enough
        if distance < 0.1:
            print(f"Reached waypoint {self.current_index + 1}: {goal}")
            self.current_index += 1

        self.publish_velocity(linear_velocity, angular_velocity)

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
        print(f"Published velocity: linear={linear}, angular={angular}")

    def run(self):
        print("Starting waypoint navigation.")
        while self.current_index < len(self.optimized_path):
            self.navigate_to_waypoint()
            time.sleep(0.1)
        print("Waypoint navigation complete.")


if __name__ == "__main__":
    controller = WaypointFollower()
    controller.run()
