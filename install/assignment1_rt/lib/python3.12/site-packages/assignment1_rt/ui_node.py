#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


class UI_Node(Node):
    def __init__(self):
        super().__init__('ui_node')

        self.publishers_dict = {
            'turtle1': self.create_publisher(Twist, '/turtle1/cmd_vel', 10),
            'turtle2': self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        }

        self.get_logger().info("UI Node Started. Ready for input.")

    def send_velocity(self, turtle_name, linear_x, angular_z):
        """
        Publishes velocity, waits 1 second, then stops the robot.
        """
        if turtle_name not in self.publishers_dict:
            self.get_logger().error(f"Invalid turtle name: {turtle_name}")
            return

        publisher = self.publishers_dict[turtle_name]

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        # Publish Move
        publisher.publish(msg)
        self.get_logger().info(f"Moving {turtle_name} for 1 second...")

        # Wait for 1 second
        time.sleep(1.0)

        # Create the Stop Command
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        # Publish Stop
        publisher.publish(stop_msg)
        self.get_logger().info(f"Stopped {turtle_name}.")


def main(args=None):
    rclpy.init(args=args)
    node = UI_Node()

    try:

        while rclpy.ok():
            print("-----------------------------------------")
            print("Enter command for the turtle.")

            # Select Robot
            turtle_choice = input("Select robot (turtle1 or turtle2): ").strip()
            if turtle_choice not in ['turtle1', 'turtle2']:
                print("Invalid choice. Please type 'turtle1' or 'turtle2'.")
                continue

            try:
                # Select Velocities
                lin_vel = float(input("Enter Linear Velocity (X): "))
                ang_vel = float(input("Enter Angular Velocity (Z): "))

                # Execute Logic
                node.send_velocity(turtle_choice, lin_vel, ang_vel)

            except ValueError:
                print("Invalid number format. Please enter numeric values for velocity.")

    except KeyboardInterrupt:
        print("\nExiting UI Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
