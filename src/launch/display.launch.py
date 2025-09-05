#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('small_ros')

    # Path to URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'Small_ROS1.urdf')

    # Optional RViz config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'small_ros.rviz')

    #controller 
    controller_file = os.path.join(pkg_share, 'config', 'test_diff_drive_controller.yaml')

    return LaunchDescription([
        # Publish the URDF as TF using robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            # parameters=[{'source_list': []}],  # empty list reads all joints from URDF
            output='screen'
        ),

        # Static TF publisher (optional, base_link -> base_footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            output='screen'
        ),        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='small_ros',
            executable='odom_raw.py',  
            name='odom_raw',
            output='screen',
            # Optional: pass serial port and baud rate as ROS parameters
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 115200}]
        )
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     parameters=[
        #         {'robot_description': open(urdf_file).read()},
        #         {'controller_file': open(controller_file).read()},
        #     ],
        #     output='screen'
        # )
    ])
