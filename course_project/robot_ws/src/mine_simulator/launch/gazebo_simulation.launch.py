#!/usr/bin/env python3
"""Gazebo simulation launch file for perimeter miner."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create Gazebo simulation launch description."""
    # Package paths
    mine_simulator_pkg = get_package_share_directory('mine_simulator')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    # SDF world file
    world_file = os.path.join(
        mine_simulator_pkg, 'sdf', 'perimeter_mining_ground.sdf'
    )

    # URDF robot file
    urdf_file = os.path.join(
        mine_simulator_pkg, 'urdf', 'perimeter_miner.urdf'
    )

    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to SDF world file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial Y position'
    )

    yaw_pose_arg = DeclareLaunchArgument(
        'yaw_pose',
        default_value='0.0',
        description='Initial yaw rotation'
    )

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': 'true',
            'verbose': 'false',
        }.items()
    )

    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
        ])
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': True,
        }]
    )

    # Spawn robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'perimeter_miner',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.5',
            '-Y', LaunchConfiguration('yaw_pose'),
        ]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param', '/diff_drive_controller.yaml',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        x_pose_arg,
        y_pose_arg,
        yaw_pose_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
    ])
