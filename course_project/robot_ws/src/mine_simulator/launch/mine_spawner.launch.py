#!/usr/bin/env python3
"""Mine spawner launch file for perimeter miner simulation.

Usage:
    ros2 launch mine_simulator mine_spawner.launch.py
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    """Generate mine spawner launch description."""
    mine_simulator_pkg = get_package_share_directory('mine_simulator')

    scenario = DeclareLaunchArgument(
        'scenario',
        default_value='training_ground',
        description='Scenario name'
    )

    mine_spawner_node = Node(
        package='mine_simulator',
        executable='mine_spawner_node',
        name='mine_spawner',
        output='screen',
        parameters=[{
            'config_path': PathJoinSubstitution([
                mine_simulator_pkg, 'config',
                LaunchConfiguration('scenario') + '_mines.yaml'
            ])
        }]
    )

    return LaunchDescription([
        scenario,
        mine_spawner_node,
    ])
