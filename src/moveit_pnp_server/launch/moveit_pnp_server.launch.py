#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_pnp_server_share = get_package_share_directory('moveit_pnp_server')

    # Аргумент для сцены
    scene_file_arg = DeclareLaunchArgument(
        'scene_file',
        default_value=os.path.join(moveit_pnp_server_share, 'meshes', 'tables.scene'),
        description='Path to the MoveIt scene file'
    )

    # Включаем MoveIt demo для Panda (поднимает robot_state_publisher + move_group + SRDF)
    panda_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_resources_panda_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ])
    )

    # Статические трансформы для pick и place столов
    pick_table_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pick_table_frame_publisher',
        arguments=['0.4', '0', '0.3', '0', '0', '0', 'world', 'pick_table']
    )

    place_table_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='place_table_frame_publisher',
        arguments=['0', '0.4', '0.3', '0', '0', '0', 'world', 'place_table']
    )

    # Публикация сцены через TimerAction (ждем 6 секунд, пока MoveIt поднимется)
    scene_publisher_node = TimerAction(
        period=6.0,
        actions=[Node(
            package='moveit_ros_planning',
            executable='moveit_publish_scene_from_text',
            name='moveit_publish_scene_from_text',
            arguments=["--scene", LaunchConfiguration('scene_file')],
            output='screen'
        )]
    )

    # Запуск basic_move после сцены (через 8 секунд)
    basic_move_node = TimerAction(
        period=8.0,
        actions=[Node(
            package='moveit_pnp_server',
            executable='basic_move',
            name='basic_move_node',
            output='screen'
        )]
    )

    return LaunchDescription([
        scene_file_arg,
        panda_moveit_launch,
        pick_table_tf,
        place_table_tf,
        scene_publisher_node,
        basic_move_node
    ])
