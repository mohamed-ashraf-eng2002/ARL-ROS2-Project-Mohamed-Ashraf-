import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class TurtleCommander(Node):
    def __init__(self):
        super().__init__("turtleCommander")
        self.vel_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.shape_subscription = self.create_subscription(
            String, "turtle_shape_command", self.command_callback, 10
        )
        self.pose_subscription = self.create_subscription(
            Pose, "turtle1/pose", self.pose_callback, 10
        )

        self.current_pose = Pose()
        self.current_shape = "stop"
        self.path = []   # list of (x,y) waypoints
        self.waypoint_idx = 0

        self.timer = self.create_timer(0.05, self.update)  # control loop
        self.get_logger().info("Turtle Commander ready. Waiting for a shape command...")

    def pose_callback(self, msg):
        self.current_pose = msg

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received command: "{command}"')

        if command == "stop":
            self.stop_turtle()
            self.current_shape = "stop"
            self.path = []
            return

        if command in ["heart", "star", "infinity", "rose"]:
            self.current_shape = command
            self.path = self.generate_shape_path(command)
            self.waypoint_idx = 0
            self.get_logger().info(f"Generated path for {command} with {len(self.path)} points")
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def stop_turtle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)

    def update(self):
        """Main control loop following waypoints"""
        if not self.path or self.current_shape == "stop":
            return

        if self.waypoint_idx >= len(self.path):
            self.get_logger().info(f"Finished drawing {self.current_shape}")
            self.stop_turtle()
            self.current_shape = "stop"
            return

        target_x, target_y = self.path[self.waypoint_idx]
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if reached waypoint
        if distance < 0.2:
            self.waypoint_idx += 1
            return

        angle_to_target = math.atan2(dy, dx)
        angular_error = angle_to_target - self.current_pose.theta
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        twist = Twist()
        twist.angular.z = 2.0 * angular_error
        twist.linear.x = min(1.5 * distance, 2.0) if abs(angular_error) < 1.0 else 0.0
        self.vel_publisher.publish(twist)

    def generate_shape_path(self, shape):
        """Generate waypoints for a given shape"""
        points = []
        center_x, center_y = 5.5, 5.5
        A = 3.0
        steps = 500

        if shape == "heart":
            for t in [i * 2 * math.pi / steps for i in range(steps)]:
                x = 16 * math.sin(t) ** 3
                y = (
                    13 * math.cos(t)
                    - 5 * math.cos(2 * t)
                    - 2 * math.cos(3 * t)
                    - math.cos(4 * t)
                )
                points.append((center_x + A * x / 30.0, center_y + A * y / 30.0))

        elif shape == "star":
            R = 3.0  # radius of star
            points_star = []
            # compute 5 points on a circle
            star_points = [
                (center_x + R * math.cos(2 * math.pi * i / 5),
                center_y + R * math.sin(2 * math.pi * i / 5))
                for i in range(5)
            ]
            # connect them in star order
            order = [0, 2, 4, 1, 3, 0]
            for i in range(len(order) - 1):
                x1, y1 = star_points[order[i]]
                x2, y2 = star_points[order[i + 1]]
                points_star.extend(self.interpolate_line(x1, y1, x2, y2, 50))
            points = points_star

        elif shape == "infinity":
            for t in [i * 2 * math.pi / steps for i in range(steps)]:
                x = A * math.sin(t)
                y = A * math.sin(t) * math.cos(t)
                points.append((center_x + x, center_y + y))

        elif shape == "rose":
            k = 5
            for t in [i * 2 * math.pi / steps for i in range(steps)]:
                r = A * math.cos(k * t)
                x, y = r * math.cos(t), r * math.sin(t)
                points.append((center_x + x, center_y + y))

        return points

    def interpolate_line(self, x1, y1, x2, y2, n_points):
        return [(x1 + (x2 - x1) * i / n_points, y1 + (y2 - y1) * i / n_points) for i in range(n_points)]


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
