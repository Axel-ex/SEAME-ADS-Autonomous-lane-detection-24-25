from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
            DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
      ),
        Node (
            package='classic_vision',
            node_executable='classic_vision_node',
            node_name='classic_vision',
            arguments=['--ros-args', '--log-level', 'debug'],
        ),
        Node (
            package='image_publisher',
            node_executable='image_publisher_node',
            node_name='image_publisher',
            arguments=['--ros-args', '--log-level', 'debug'],
        ),
        # Node (
        #     package='camera',
        #     node_executable='camera_node',
        #     node_name='camera_node',
        #     arguments=['--ros-args', '--log-level', 'debug'],
        # )
    ])
