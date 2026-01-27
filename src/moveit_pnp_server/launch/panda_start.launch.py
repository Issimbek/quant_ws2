#!/usr/bin/env python3
"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction

def generate_launch_description():

    panda_moveit_config_path = get_package_share_directory("moveit_resources_panda_moveit_config")
    panda_moveit_description_path = get_package_share_directory("moveit_resources_panda_description")
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(
            file_path=os.path.join(panda_moveit_description_path, "config", "panda.urdf.xacro")
        )
        .trajectory_execution(
            file_path=os.path.join(
                panda_moveit_config_path, "config", "gripper_moveit_controllers.yaml"
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(panda_moveit_config_path, "config", "panda.srdf")
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("moveit_pnp_server"),
                "config",
                "motion_planning_python_api_tutorial.yaml",
            )
        )
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    

    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_pnp_server"),
        "config",
        "motion_planning_python_api_tutorial.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )


    ros2_controllers_path = os.path.join(
        panda_moveit_config_path,
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )
    basic_move_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="moveit_pnp_server",
                executable="basic_move",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                ],
            )
        ]
    )

    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {controller}"],
                shell=True,
                output="log",
            )
        ]

    return LaunchDescription(
        [
            move_group,
            basic_move_node,
            robot_state_publisher,
            ros2_control_node,
            rviz_node,
            static_tf,
        ]
        + load_controllers
    )
