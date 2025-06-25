from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the drone survey node."""
    
    # Declare launch arguments
    road_gradient = DeclareLaunchArgument(
        'road_gradient',
        default_value='0.03',
        description='Maximum road gradient for traversability (e.g., 0.03 for 3%)'
    )
    
    advanced = DeclareLaunchArgument(
        'advanced',
        default_value='false',
        description='Enable advanced planning mode (TSP optimization)'
    )
    
    # Create drone survey node
    drone_node = Node(
        package='a3_skeleton',
        executable='drone_node',
        name='drone_node',
        output='screen',
        parameters=[{
            'road_gradient': LaunchConfiguration('road_gradient'),
            'advanced': LaunchConfiguration('advanced')
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        road_gradient,
        advanced,
        drone_node
    ])