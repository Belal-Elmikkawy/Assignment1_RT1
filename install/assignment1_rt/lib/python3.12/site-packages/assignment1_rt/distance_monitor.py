import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Import the logic file we just created
from assignment1_rt.distance_checks import TurtleChecks


class DistanceMonitor(Node):
    def __init__(self):
        super().__init__('distance_monitor')

        # 1. Variables to store latest positions
        self.pose_t1 = None
        self.pose_t2 = None

        # 2. Subscribers (Get positions)
        self.create_subscription(Pose, '/turtle1/pose', self.update_pose_t1, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.update_pose_t2, 10)

        # 3. Publishers (Send data and commands)
        self.pub_distance = self.create_publisher(Float32, '/turtles_distance', 10)
        self.pub_cmd_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_cmd_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # 4. Control Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Distance Monitor Node Started.")

    def update_pose_t1(self, msg):
        self.pose_t1 = msg

    def update_pose_t2(self, msg):
        self.pose_t2 = msg

    def stop_turtle(self, turtle_name):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        if turtle_name == 'turtle1':
            self.pub_cmd_t1.publish(stop_msg)
        elif turtle_name == 'turtle2':
            self.pub_cmd_t2.publish(stop_msg)

        self.get_logger().warn(f"STOPPING {turtle_name} (Safety Breach)")

    def control_loop(self):
        # Wait until we have data for both turtles
        if self.pose_t1 is None or self.pose_t2 is None:
            return

        # --- A. Calculate and Publish Distance ---
        dist = TurtleChecks.calculate_distance(self.pose_t1, self.pose_t2)

        dist_msg = Float32()
        dist_msg.data = dist
        self.pub_distance.publish(dist_msg)

        # --- B. Check Collision (Too Close) ---
        if TurtleChecks.is_too_close(dist, threshold=2.0):
            self.get_logger().warn(f"Turtles too close! Dist: {dist:.2f}")
            self.stop_turtle('turtle1')
            self.stop_turtle('turtle2')

        # --- C. Check Boundaries ---
        if TurtleChecks.is_near_boundary(self.pose_t1):
            self.get_logger().warn("Turtle1 hitting boundary!")
            self.stop_turtle('turtle1')

        if TurtleChecks.is_near_boundary(self.pose_t2):
            self.get_logger().warn("Turtle2 hitting boundary!")
            self.stop_turtle('turtle2')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
