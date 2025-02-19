import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument('service_name', default_value='llm_service'),
        DeclareLaunchArgument('timeout', default_value='10.0'),
        DeclareLaunchArgument('max_connections', default_value='5'),

       
        Node(
            package='llm_control_dev',
            executable='llm_service_node',
            name='llm_service_node',
            output='screen',
            parameters=[{
                'service_name': 'llm_service',
                'timeout': 10.0,
                'max_connections': 5,
            }],
        ),
        
        
        Node(
            package='llm_control_dev',
            executable='llm_control_node.py',
            name='llm_control_node',
            output='screen',
        ),
        

    ])
