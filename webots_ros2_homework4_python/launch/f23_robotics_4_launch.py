#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots TurtleBot3 Burger driver."""

import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_homework4_python')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners
    )

    # Camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen'
    )

    # AprilTag detection node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info')
        ],
        parameters=[os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11.yaml')]
    )

    # RQT Image View node
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='screen'
    )

    # Echo detections topic
    detection_echo = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/detections'],
        output='screen'
    )

    image_color_echo = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/TurtleBot3Burger/camera/image_color'],
        output='screen'
    )

    rviz_config_dir = os.path.join(get_package_share_directory('webots_ros2_homework4_python'),
                                   'rviz', 'turtlebot3_apriltags.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_apriltags.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,
        waiting_nodes,
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        camera_node,
        apriltag_node,
        rqt_image_view_node,
        detection_echo,
        image_color_echo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])
