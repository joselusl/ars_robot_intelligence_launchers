#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare





def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    ars_path_follower_node_name_arg = DeclareLaunchArgument(
        'ars_path_follower_node_name', default_value='ars_path_follower_node',
        description='Name of the node'
    )

    ars_path_follower_yaml_file_arg=DeclareLaunchArgument(
      'config_param_path_follower_yaml_file',
      default_value=PathJoinSubstitution(['path_follower', 'config_path_follower.yaml']), 
      description='Path to the config_param_path_follower_yaml_file'
    )

    flag_use_state_estim_arg = DeclareLaunchArgument(
        'flag_use_state_estim', default_value='False',
        description='flag_use_state_estim'
    )

    def topics_state_estim(context, *args, **kwargs):
      flag_use_state_estim = LaunchConfiguration('flag_use_state_estim').perform(context).lower()
      if(flag_use_state_estim == 'true'):
        robot_pose_topic_value = '/estim_robot_pose'
        robot_velocity_world_topic_value = '/estim_robot_velocity_world'
      else:
        robot_pose_topic_value = '/simulator/sim_robot/robot_pose'
        robot_velocity_world_topic_value = '/simulator/sim_robot/robot_velocity_world'
        
      robot_pose_topic_arg = DeclareLaunchArgument(
        'robot_pose_topic', default_value=robot_pose_topic_value,
        description='Topic robot_pose')

      robot_velocity_world_topic_arg = DeclareLaunchArgument(
        'robot_velocity_world_topic', default_value=robot_velocity_world_topic_value,
        description='Topic robot_velocity_world')

      return [robot_pose_topic_arg, robot_velocity_world_topic_arg]
    
    topics_state_estim_funct=OpaqueFunction(function=topics_state_estim)

    
    robot_pose_ctr_ref_topic_arg = DeclareLaunchArgument(
          'robot_pose_ctr_ref_topic', default_value='/robot_pose_ref',
          description='Topic robot_pose_ctr_ref')
    
    robot_velocity_world_ctr_ref_topic_arg = DeclareLaunchArgument(
          'robot_velocity_world_ctr_ref_topic', default_value='/robot_velocity_world_ref',
          description='Topic robot_velocity_world_ctr_ref')
    
    robot_ctr_cmd_ref_topic_arg = DeclareLaunchArgument(
          'robot_ctr_cmd_ref_topic', default_value='/robot_cmd_ref',
          description='Topic robot_ctr_cmd_ref')

    robot_trajectory_ref_topic_arg = DeclareLaunchArgument(
          'robot_trajectory_ref_topic', default_value='/robot_trajectory_ref',
          description='Topic robot_trajectory_ref')


    # Get the launch configuration for parameters
    ars_path_follower_conf_yaml_file = PathJoinSubstitution([FindPackageShare('ars_robot_intelligence_config'), 'config', LaunchConfiguration('config_param_path_follower_yaml_file')])
    

    # Define the nodes
    ars_path_follower_node = Node(
        package='ars_path_follower',
        executable='ars_path_follower_ros_node',
        name=LaunchConfiguration('ars_path_follower_node_name'),
        output=LaunchConfiguration('screen'),
        parameters=[{'config_param_path_follower_yaml_file': ars_path_follower_conf_yaml_file}],
        remappings=[
          ('robot_pose', LaunchConfiguration('robot_pose_topic')),
          ('robot_velocity_world', LaunchConfiguration('robot_velocity_world_topic')),
          ('robot_pose_ctr_ref', LaunchConfiguration('robot_pose_ctr_ref_topic')),
          ('robot_velocity_world_ctr_ref', LaunchConfiguration('robot_velocity_world_ctr_ref_topic')),
          ('robot_ctr_cmd_ref', LaunchConfiguration('robot_ctr_cmd_ref_topic')),
          ('robot_trajectory_ref', LaunchConfiguration('robot_trajectory_ref_topic')),
        ]
    )


    return LaunchDescription([
        screen_arg,
        ars_path_follower_node_name_arg,
        ars_path_follower_yaml_file_arg,
        flag_use_state_estim_arg,
        topics_state_estim_funct,
        robot_pose_ctr_ref_topic_arg,
        robot_velocity_world_ctr_ref_topic_arg,
        robot_ctr_cmd_ref_topic_arg,
        robot_trajectory_ref_topic_arg,
        GroupAction([
          PushRosNamespace('robot_intelligence'),
          GroupAction([
            PushRosNamespace('path_follower'),
            ars_path_follower_node,
          ]),
        ]),
    ])
