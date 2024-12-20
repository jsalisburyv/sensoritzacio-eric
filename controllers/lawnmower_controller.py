import time
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.pose_v_pb2 import Pose_V
import math
from model_deleter import ModelDeleter
import re


class LawnMowerController:
    def __init__(self):
        self.deleter = ModelDeleter()
        self.node = Node()
        self.publisher = self.node.advertise("/cmd_vel", Twist)
        self.pose_subscriber = self.node.subscribe(msg_type=Pose_V, topic="/world/beach_world/pose/info", callback=self.pose_callback)

        # Parameters for movement
        self.distance = 15
        self.speed = 1.0  # Linear speed
        self.turn_speed = 1.0  # Angular speed
        self.current_position = [0.0, 0.0]
        self.current_orientation = None  # Robot's current orientation (yaw)

    @staticmethod
    def euclidean_distance(a, b):
        """
        Compute Euclidean distance between two points.
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def pose_callback(self, msg):
        for pose in msg.pose:
            if pose.name == "MR-Buggy3":  # Replace with your robot's model name
                self.current_position = [pose.position.x, pose.position.y]
                # Extract the yaw (rotation about Z-axis) from the orientation
                qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                self.current_orientation = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                print(f"Current orientation (yaw): {self.current_orientation}")

            model_name = pose.name
            if re.match(r"can_\d+", model_name):
                model_position = [pose.position.x, pose.position.y]
                distance = self.euclidean_distance(self.current_position, model_position)
                if distance < 0.5:  # Collision threshold
                    print(f"Collision detected with {model_name}. Removing it...")
                    self.deleter.delete_model(model_name)

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        #print(f"Published velocity: linear={linear}, angular={angular}")

    def turn_180(self, direction):
        print("Starting hardcoded 180-degree turn with linear velocity.")

        # Duration for 180-degree turn
        duration = 2.3 * math.pi / self.turn_speed * 0.85  # Time to turn 180 degrees at constant angular velocity
        start_time = time.time()

        while time.time() - start_time < duration:
            # Apply both linear and angular velocity for a realistic turn
            self.publish_velocity(0.5, self.turn_speed if direction else -self.turn_speed)  # Adjust linear velocity as needed
            time.sleep(0.1)

        # Stop the robot after completing the turn
        self.publish_velocity(0, 0)


    def move_straight(self):
        #print(f"Moving straight for {distance} units at {speed} speed.")
        duration = self.distance / self.speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publish_velocity(self.speed, 0)
            time.sleep(0.1)

        self.publish_velocity(0, 0)
        print("Straight movement complete.")

    def run(self):
        while True:
            self.move_straight()
            self.turn_180(direction=False)
            self.move_straight()
            self.turn_180(direction=True)


if __name__ == "__main__":
    controller = LawnMowerController()
    controller.run()
