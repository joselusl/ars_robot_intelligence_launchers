#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    ars_remote_controller_node_name_arg = DeclareLaunchArgument(
        'ars_remote_controller_node_name', default_value='ars_remote_controller_node',
        description='Name of the remote controller node'
    )

    robot_cmd_stamped_topic_arg = DeclareLaunchArgument(
        'robot_cmd_stamped_topic', default_value='/robot_cmd_stamped',
        description='Topic robot_cmd_stamped'
    )

    # Define the nodes
    ars_remote_controller_node = Node(
        package='ars_remote_controller',
        executable='ars_remote_controller_ros_node',
        name=LaunchConfiguration('ars_remote_controller_node_name'),
        output=LaunchConfiguration('screen'),
        prefix='gnome-terminal --',
        remappings=[
          ('robot_cmd_stamped', LaunchConfiguration('robot_cmd_stamped_topic')),
        ]
    )


    return LaunchDescription([
        screen_arg,
        ars_remote_controller_node_name_arg,
        robot_cmd_stamped_topic_arg,
        GroupAction([
          PushRosNamespace('remote_controller'),
          ars_remote_controller_node,
        ]),
    ])
