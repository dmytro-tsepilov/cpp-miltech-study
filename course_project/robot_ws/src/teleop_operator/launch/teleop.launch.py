#!/usr/bin/env python3
"""Teleop operator launch file - keyboard/gamepad input for perimeter miner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create teleop operator launch description."""
    # Launch arguments
    input_type_arg = DeclareLaunchArgument(
        'input_type',
        default_value='keyboard',
        description='Input type: keyboard or joy'
    )

    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='1.0',
        description='Max linear speed (m/s)'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='1.5',
        description='Max angular speed (rad/s)'
    )

    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.1',
        description='Deadzone for analog inputs'
    )

    # Teleop node
    teleop_node = Node(
        package='teleop_operator',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[{
            'input_type': LaunchConfiguration('input_type'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
            'deadzone': LaunchConfiguration('deadzone'),
        }]
    )

    return LaunchDescription([
        input_type_arg,
        max_linear_arg,
        max_angular_arg,
        deadzone_arg,
        teleop_node,
    ])
