from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Matches "robot_ip" in your controller.cpp
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the robot'
    )
    
    username_arg = DeclareLaunchArgument(
        'username',
        default_value='admin',
        description='Username for robot connection'
    )
    
    password_arg = DeclareLaunchArgument(
        'password',
        default_value='admin',
        description='Password for robot connection'
    )

    # Note: inactivity timeouts are not declared in your provided controller.cpp
    # Passing them here will be ignored unless you add declare_parameter() for them in C++.

    controller_node = Node(
        package='kortex_controller',
        executable='controller',
        name='kinova_controller',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'), # Changed from ip_address
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
        }]
    )

    return LaunchDescription([
        robot_ip_arg,
        username_arg,
        password_arg,
        controller_node
    ])