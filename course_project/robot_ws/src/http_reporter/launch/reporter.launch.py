#!/usr/bin/env python3
"""HTTP reporter launch file for perimeter miner.

Usage:
    ros2 launch http_reporter reporter.launch.py
    ros2 launch http_reporter reporter.launch.py
        api_endpoint:=http://192.168.1.100:8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""
    api_endpoint_arg = DeclareLaunchArgument(
        'api_endpoint',
        default_value='http://localhost:8080',
        description='HTTP API endpoint for reports'
    )

    timeout_ms_arg = DeclareLaunchArgument(
        'timeout_ms',
        default_value='5000',
        description='Request timeout in milliseconds'
    )

    reporter_node = Node(
        package='http_reporter',
        executable='reporter_node',
        name='reporter_node',
        output='screen',
        parameters=[{
            'api_endpoint': LaunchConfiguration('api_endpoint'),
            'timeout_ms': LaunchConfiguration('timeout_ms'),
        }]
    )

    return LaunchDescription([
        api_endpoint_arg,
        timeout_ms_arg,
        reporter_node,
    ])
