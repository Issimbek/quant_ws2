"""
ROS2 Launch file for MoveIt Pick and Place Server.

This launch file:
1. Includes the Panda MoveIt demo launch
2. Loads the scene geometry from the .scene file
3. Publishes static transforms for pick and place tables
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Get package share directories
    moveit_pnp_server_share = get_package_share_directory('moveit_pnp_server')

    # Declare launch arguments
    scene_file_arg = DeclareLaunchArgument(
        'scene_file',
        default_value=os.path.join(moveit_pnp_server_share, 'meshes', 'tables.scene'),
        description='Path to the MoveIt scene file'
    )

    # Include the Panda MoveIt demo launch file
    # Note: For ROS2, you'll need to use the appropriate MoveIt2 panda config package
    # The package name may vary - common options:
    # - moveit_resources_panda_moveit_config (from moveit_resources)
    # - panda_moveit_config (if separately installed)
    panda_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_resources_panda_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ])
    )

    # Node to publish the scene from text file
    # In ROS2/MoveIt2, this is typically done differently
    # You may need to use the moveit_ros_planning publish_scene_from_text node
    # or load the scene programmatically
    scene_publisher_node = Node(
        package='moveit_ros_planning',
        executable='moveit_publish_scene_from_text',
        name='moveit_publish_scene_from_text',
        arguments=["--scene", LaunchConfiguration('scene_file')],
        output='screen'
    )

    # Static transform publishers for pick and place table frames
    # In ROS2, static_transform_publisher uses different argument format
    pick_table_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pick_table_frame_publisher',
        arguments=[
            '--x', '0.4',
            '--y', '0',
            '--z', '0.3',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'pick_table'
        ]
    )

    place_table_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='place_table_frame_publisher',
        arguments=[
            '--x', '0',
            '--y', '0.4',
            '--z', '0.3',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'place_table'
        ]
    )

    movable_box_node = Node(
        package='moveit_pnp_server',          
        executable='spawn_movable_box.py',       
        name='spawn_movable_box',
        output='screen'
    )
    
    # Uncomment when you have the server node implemented
    # moveit_pnp_server_node = Node(
    #     package='moveit_pnp_server',
    #     executable='moveit_pnp_server_node',
    #     name='moveit_pnp_server_node',
    #     output='screen'
    # )

    return LaunchDescription([
        scene_file_arg,
        panda_moveit_launch,
        scene_publisher_node,
        pick_table_tf,
        place_table_tf,
        movable_box_node
        # moveit_pnp_server_node,
    ])