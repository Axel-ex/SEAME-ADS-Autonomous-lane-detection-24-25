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
            package='classic_vision',
            node_executable='classic_vision_node',
            node_name='classic_vision',
        ),
        Node(
            package='image_publisher',
            node_executable='image_publisher_node',  
            node_name='image_publisher',  
        ),
        Node(
            package='motion_control',
            node_executable='motion_control_node',  
            node_name='motion_control',  
        ),
        # Node(
        #     package='lane_visualization',
        #     node_executable='lane_visualization_node',  
        #     node_name='lane_visualization',  
        # ),
        # Node(
        #     package='camera',
        #     executable='camera_node',  
        #     name='camera_node',  
        #     arguments=['--ros-args', '--log-level', 'debug'],
        # )
    ])
