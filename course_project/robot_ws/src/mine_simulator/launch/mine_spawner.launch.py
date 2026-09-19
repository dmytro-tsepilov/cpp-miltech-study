#!/usr/bin/env python3
"""Mine spawner launch file for perimeter miner simulation.

Usage:
    ros2 launch mine_simulator mine_spawner.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _launch_context_setup(context, *args, **kwargs):
    """Opaque function to set up mine spawner with correct config path."""
    scenario_name = LaunchConfiguration('scenario').perform(context)
    
    # Map scenario names to config files
    scenario_map = {
        'training_ground': 'training_ground_mines.yaml',
        'patrol_alpha': 'training_ground_mines.yaml',
        'large_patrol': 'training_ground_mines.yaml',
    }
    
    config_file = scenario_map.get(scenario_name, f'{scenario_name}_mines.yaml')
    config_path = os.path.join(
        get_package_share_directory('mine_simulator'),
        'config',
        config_file
    )
    
    # Create node with computed config path
    mine_spawner_node = Node(
        package='mine_simulator',
        executable='mine_spawner_node',
        name='mine_spawner',
        output='screen',
        parameters=[{
            'config_path': config_path,
        }]
    )
    
    return [mine_spawner_node]


def generate_launch_description():
    """Generate mine spawner launch description."""
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='training_ground',
        description='Scenario name'
    )

    return LaunchDescription([
        scenario_arg,
        OpaqueFunction(function=_launch_context_setup),
    ])
