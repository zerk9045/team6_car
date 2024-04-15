#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_car = os.path.join(get_package_share_directory('team6_car'))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Spawn the car entity
    car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_car, 'launch', 'spawn_car.launch.py'),
        )
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        output='screen')

    teleop_twist = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        parameters=[os.path.join(pkg_sim_car, "config", "teleop_twist_joy.yaml")])
    
    ackermann_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["asc"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jsc"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_sim_car, 'worlds', 'simple.world'), ''],
            description='SDF world file'),
        gazebo,
        car,
        joy,
        teleop_twist,
        ackermann_drive_spawner,
        joint_broad_spawner
    ])