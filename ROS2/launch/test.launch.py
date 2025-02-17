#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the parameter "my_param"
    declare_my_param = DeclareLaunchArgument(
        'my_param',
        default_value='launch_default_value',
        description='Parameter for MyNode'
    )

    # Define the node using the declared parameter
    my_node = Node(
        package='ros2_test',      # Replace with your package name
        executable='ros2_test',      # Replace with your node executable name
        name='my_node',            # Node name as defined in your code
        output='screen',
        parameters=[{'my_param': LaunchConfiguration('my_param')}]
    )

    # Create and return the launch description
    return LaunchDescription([
        declare_my_param,
        my_node
    ])
