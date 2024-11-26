import time
import sys
import termios
import tty
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist

class InteractiveKeyboardController:
    def __init__(self):
        self.node = Node()
        # Gazebo Transport Publisher for /cmd_vel
        self.publisher = self.node.advertise("/cmd_vel", Twist)

        # Speeds
        self.linear_speed = 0.5  # Speed for forward/backward motion
        self.angular_speed = 0.5  # Speed for turning

        # Current velocities
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        print("Interactive Keyboard Controller Initialized!")
        print("Use W/A/S/D to control the robot, SPACE to stop, and Q to quit.")

    def publish_velocity(self):
        """Publishes the current linear and angular velocities."""
        msg = Twist()
        msg.linear.x = self.current_linear_velocity
        msg.angular.z = self.current_angular_velocity
        self.publisher.publish(msg)

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
        print("Ready to receive commands:")
        print("[W]: Forward  [S]: Backward  [A]: Turn Left  [D]: Turn Right")
        print("[SPACE]: Stop  [Q]: Quit")

        try:
            while True:
                key = self.get_key().lower()

                if key == 'w':  # Forward
                    self.current_linear_velocity += self.linear_speed
                    print("Moving forward")

                elif key == 's':  # Backward
                    self.current_linear_velocity -= self.linear_speed
                    print("Moving backward")

                elif key == 'a':  # Turn Left
                    self.current_angular_velocity += self.angular_speed
                    print("Turning left")

                elif key == 'd':  # Turn Right
                    self.current_angular_velocity -= self.angular_speed
                    print("Turning right")

                elif key == ' ':  # Stop
                    self.current_linear_velocity = 0.0
                    self.current_angular_velocity = 0.0
                    print("Stopped")

                elif key == 'q':  # Quit
                    print("Quitting. Stopping the robot.")
                    self.current_linear_velocity = 0.0
                    self.current_angular_velocity = 0.0
                    self.publish_velocity()
                    break

                else:
                    print("Invalid key! Use W/A/S/D/SPACE/Q.")

                self.publish_velocity()

        except KeyboardInterrupt:
            print("\nExiting controller.")
            self.current_linear_velocity = 0.0
            self.current_angular_velocity = 0.0
            self.publish_velocity()

if __name__ == "__main__":
    controller = InteractiveKeyboardController()
    controller.run()
