#!/usr/bin/env python3
# Copyright 2026 Open Source Robotics Foundation Inc
# SPDX-License-Identifier: MIT

"""Launch fake odometry publisher for testing without Gazebo.

This node simulates robot movement along a perimeter path, publishing:
- /odom (nav_msgs/Odometry)
- /robot/pose (geometry_msgs/PoseWithCovarianceStamped)
- /robot/position (std_msgs/String) - for mine_spawner

Usage:
    ros2 launch mine_simulator fake_odom_publisher.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""

    # Fake odom publisher node with default training_ground perimeter
    fake_odom_node = Node(
        package='mine_simulator',
        executable='fake_odom_publisher',
        name='fake_odom_publisher',
        output='screen',
        parameters=[{
            'waypoints_flat': [0.0, 0.0, 20.0, 0.0, 20.0, 20.0, 0.0, 20.0],
            'speed': 2.0,
            'publish_rate_hz': 10.0,
            'use_robot_position_topic': True,
        }]
    )

    return LaunchDescription([
        fake_odom_node,
    ])
