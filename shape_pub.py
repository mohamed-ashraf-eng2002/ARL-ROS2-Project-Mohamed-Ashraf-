import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class ShapePublisher(Node):
    """Publishes a shape command only once."""

    def __init__(self, initial_shape: str):
        super().__init__("shapeNode")
        self.publisher_ = self.create_publisher(String, "turtle_shape_command", 10)

        self.current_shape = initial_shape.lower()
        self.get_logger().info(f"Publishing one-time shape command: {self.current_shape}")

        # Publish once after a short delay (so subscriber exists)
        self.timer = self.create_timer(0.5, self.publish_once)

    def publish_once(self):
        msg = String()
        msg.data = self.current_shape
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: "{self.current_shape}"')

        # Cancel timer so it doesn’t repeat
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    # Get launch argument (sys.argv[1] is the shape from launch file)
    initial_shape = "stop"
    if len(sys.argv) > 1:
        initial_shape = sys.argv[1]

    node = ShapePublisher(initial_shape)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
