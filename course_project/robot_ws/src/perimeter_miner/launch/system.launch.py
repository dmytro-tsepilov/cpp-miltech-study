#!/usr/bin/env python3
# Copyright 2026 Open Source Robotics Foundation Inc
# SPDX-License-Identifier: MIT

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""System launch file for perimeter miner course project.

Launches all nodes required for perimeter patrol mission:
- miner_node: Main perimeter tracking controller
- mode_switch_node: Mode switching handler
- mine_spawner_node: Mine simulation (optional)
- reporter_node: HTTP reporting (optional)
- teleop_node: Teleoperation operator (optional)
- Gazebo simulation (optional)

Usage:
    ros2 launch perimeter_miner system.launch.py
    ros2 launch perimeter_miner system.launch.py
        scenario:=training_ground
    ros2 launch perimeter_miner system.launch.py
        simulate_mines:=true
    ros2 launch perimeter_miner system.launch.py use_gazebo:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _get_scenario_file(scenario_name):
    """Helper to get scenario file from scenario name."""
    # Map scenario names to config files
    scenario_map = {
        'training_ground': 'training_ground.yaml',
        'patrol_alpha': 'patrol_alpha.yaml',
        'large_patrol': 'large_patrol.yaml',
    }
    return scenario_map.get(scenario_name, f'{scenario_name}.yaml')


def generate_launch_description():
    """Generate the system launch description."""
    # Get package paths
    mine_simulator_pkg = get_package_share_directory('mine_simulator')
    teleop_operator_pkg = get_package_share_directory('teleop_operator')

    # Launch configuration parameters
    _scenario = LaunchConfiguration('scenario')
    simulate_mines = LaunchConfiguration('simulate_mines')
    enable_reporter = LaunchConfiguration('enable_reporter')
    api_endpoint = LaunchConfiguration('api_endpoint')
    use_gazebo = LaunchConfiguration('use_gazebo')
    enable_teleop = LaunchConfiguration('enable_teleop')

    # Scenario config path - maps scenario name to actual yaml file
    _scenario_file = LaunchConfiguration('scenario_file')

    # Default scenario file based on default scenario value
    default_scenario_file = _get_scenario_file('training_ground')

    scenario_file_arg = DeclareLaunchArgument(
        'scenario_file',
        default_value=default_scenario_file,
        description='Perimeter configuration file'
    )

    # Scenario name argument (maps to config file)
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='training_ground',
        description='Scenario name (maps to config file)'
    )

    # Simulate mines argument
    simulate_mines_arg = DeclareLaunchArgument(
        'simulate_mines',
        default_value='false',
        description='Enable mine simulation'
    )

    # Enable reporter argument
    enable_reporter_arg = DeclareLaunchArgument(
        'enable_reporter',
        default_value='false',
        description='Enable HTTP reporter node'
    )

    # API endpoint argument
    api_endpoint_arg = DeclareLaunchArgument(
        'api_endpoint',
        default_value='http://localhost:8080',
        description='HTTP API endpoint for reports'
    )

    # Use Gazebo simulation argument
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Enable Gazebo simulation'
    )

    # Enable teleop argument
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Enable teleoperation operator'
    )

    # Teleop input type argument
    teleop_input_type_arg = DeclareLaunchArgument(
        'teleop_input_type',
        default_value='keyboard',
        description='Teleop input type: keyboard or joy'
    )

    # Miner node
    miner_node = Node(
        package='perimeter_miner',
        executable='miner_node',
        name='miner_node',
        output='screen',
        parameters=[{
            'scenario_file': _scenario_file,
        }]
    )

    # Mode switch node
    mode_switch_node = Node(
        package='perimeter_miner',
        executable='mode_switch_node',
        name='mode_switch_node',
        output='screen'
    )

    # Mine spawner node (conditional)
    mine_spawner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                mine_simulator_pkg, 'launch', 'mine_spawner.launch.py'
            )
        ]),
        launch_arguments={
            'scenario': _scenario,
        }.items(),
        condition=IfCondition(simulate_mines)
    )

    # HTTP reporter node (conditional)
    reporter_node = Node(
        package='http_reporter',
        executable='reporter_node',
        name='reporter_node',
        output='screen',
        parameters=[{
            'api_endpoint': api_endpoint,
            'timeout_ms': 5000,
        }],
        condition=IfCondition(enable_reporter)
    )

    # Teleop operator node (conditional)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                teleop_operator_pkg, 'launch', 'teleop.launch.py'
            )
        ]),
        launch_arguments={
            'input_type': LaunchConfiguration('teleop_input_type'),
        }.items(),
        condition=IfCondition(enable_teleop)
    )

    # Gazebo simulation (conditional)
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                mine_simulator_pkg, 'launch', 'gazebo_simulation.launch.py'
            )
        ]),
        condition=IfCondition(use_gazebo)
    )

    return LaunchDescription([
        scenario_file_arg,
        scenario_arg,
        simulate_mines_arg,
        enable_reporter_arg,
        api_endpoint_arg,
        use_gazebo_arg,
        enable_teleop_arg,
        teleop_input_type_arg,

        miner_node,
        mode_switch_node,
        mine_spawner_launch,
        reporter_node,
        teleop_launch,
        gazebo_simulation,
    ])
