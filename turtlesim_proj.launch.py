from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument for shape
        DeclareLaunchArgument(
            "shape",
            default_value="stop",
            description="Initial shape for the turtle to draw"
        ),

        # Turtlesim node
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node"
        ),

        # Shape publisher node with launch arg
        Node(
            package="turtlesim_proj",
            executable="shape_pub",
            name="shape_pub",
            output="screen",
            emulate_tty=True,
            arguments=[LaunchConfiguration("shape")]  # <--- pass shape arg
        ),

        # Turtle Commander
        Node(
            package="turtlesim_proj",
            executable="turtle_commander",
            name="turtle_commander"
        )
    ])
