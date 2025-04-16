from launch import LaunchDescription, substitutions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    logger = substitutions.LaunchConfiguration("log_level")
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="DEBUG",
            description="Logging level",
        ),
        Node(
            package='ml_vision',
            node_executable='ml_vision_node',
            node_name='ml_vision',
        ),
        Node(
            package='motion_control',
            node_executable='motion_control_node',  
            node_name='motion_control',  
        ),
        Node(
            package='lane_visualization',
            node_executable='lane_visualization_node',  
            node_name='lane_visualization',  
        ),
        Node(
            package='camera',
            node_executable='camera_node',  
            node_name='camera_node',  
            arguments=['--ros-args', '--log-level', 'debug'],
        )
    ])
