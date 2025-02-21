from launch import LaunchDescription, substitutions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    logger = substitutions.LaunchConfiguration("log_level")
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="DEBUG",  # Should be a string, not a list
            description="Logging level",
        ),
        Node(
            package='classic_vision',
            node_executable='classic_vision_node',  # Changed from node_executable to executable
            node_name='classic_vision',  # Changed from node_name to name
            arguments=['--ros-args', '--log-level', logger],
        ),
        Node(
            package='image_publisher',
            node_executable='image_publisher_node',  # Changed from node_executable to executable
            node_name='image_publisher',  # Changed from node_name to name
            arguments=['--ros-args', '--log-level', logger],
        ),
        # Node(
        #     package='camera',
        #     executable='camera_node',  # Changed from node_executable to executable
        #     name='camera_node',  # Changed from node_name to name
        #     arguments=['--ros-args', '--log-level', 'debug'],
        # )
    ])
