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

    ars_path_planner_core_node_name_arg = DeclareLaunchArgument(
        'ars_path_planner_core_node_name', default_value='ars_path_planner_core_node',
        description='Name of the node'
    )

    ars_path_planner_manager_node_name_arg = DeclareLaunchArgument(
        'ars_path_planner_manager_node_name', default_value='ars_path_planner_manager_node',
        description='Name of the node'
    )

    config_param_path_planner_core_yaml_file_arg=DeclareLaunchArgument(
      'config_param_path_planner_core_yaml_file',
      default_value=PathJoinSubstitution(['path_planner', 'config_path_planner_core_sim_env.yaml']), 
      description='Path to the config_param_path_planner_core_yaml_file'
    )

    flag_use_state_estim_arg = DeclareLaunchArgument(
        'flag_use_state_estim', default_value='False',
        description='flag_use_state_estim'
    )

    flag_use_slam_map_arg = DeclareLaunchArgument(
        'flag_use_slam_map', default_value='False',
        description='flag_use_slam_map'
    )

    def topics_state_estim(context, *args, **kwargs):
      flag_use_state_estim = LaunchConfiguration('flag_use_state_estim').perform(context).lower()
      if(flag_use_state_estim == 'true'):
        robot_pose_topic_value = '/estim_robot_pose'
      else:
        robot_pose_topic_value = '/simulator/sim_robot/robot_pose'
        
      robot_pose_topic_arg = DeclareLaunchArgument(
        'robot_pose_topic', default_value=robot_pose_topic_value,
        description='Topic robot_pose')

      return [robot_pose_topic_arg]
    
    topics_state_estim_funct=OpaqueFunction(function=topics_state_estim)


    def topics_map(context, *args, **kwargs):
      flag_use_slam_map = LaunchConfiguration('flag_use_slam_map').perform(context).lower()
      if(flag_use_slam_map == 'true'):
          obstacles_world_topic_value = '/estim_map_world'
      else:
          obstacles_world_topic_value = '/estim_map_world_sim'

      obstacles_world_topic_arg = DeclareLaunchArgument(
          'obstacles_world_topic', default_value=obstacles_world_topic_value,
          description='Topic obstacles_world_topic')
      

      return [obstacles_world_topic_arg]

    topics_map_funct = OpaqueFunction(function=topics_map)
    

    robot_pose_des_topic_arg = DeclareLaunchArgument(
          'robot_pose_des_topic', default_value='/robot_pose_des',
          description='Topic robot_pose_des')
    
    robot_trajectory_ref_topic_arg = DeclareLaunchArgument(
          'robot_trajectory_ref_topic', default_value='/robot_trajectory_ref',
          description='Topic robot_trajectory_ref')
    
    robot_trajectory_ref_raw_topic_arg = DeclareLaunchArgument(
          'robot_trajectory_ref_raw_topic', default_value='/robot_trajectory_ref_raw',
          description='Topic robot_trajectory_ref_raw')


    # Get the launch configuration for parameters
    ars_path_planner_conf_yaml_file = PathJoinSubstitution([FindPackageShare('ars_robot_intelligence_config'), 'config', LaunchConfiguration('config_param_path_planner_core_yaml_file')])
    

    # Define the nodes
    ars_path_planner_core_node = Node(
        package='ars_path_planner_src',
        executable='ars_path_planner_core_ros_node',
        name=LaunchConfiguration('ars_path_planner_core_node_name'),
        output=LaunchConfiguration('screen'),
        parameters=[{'config_param_path_planner_core_yaml_file': ars_path_planner_conf_yaml_file}],
        remappings=[
          ('obstacles_world', LaunchConfiguration('obstacles_world_topic')),
        ]
    )

    ars_path_planner_manager_node = Node(
        package='ars_path_planner_src',
        executable='ars_path_planner_manager_ros_node',
        name=LaunchConfiguration('ars_path_planner_manager_node_name'),
        output=LaunchConfiguration('screen'),
        remappings=[
          ('robot_pose', LaunchConfiguration('robot_pose_topic')),
          ('robot_pose_des', LaunchConfiguration('robot_pose_des_topic')),
          ('robot_trajectory_ref', LaunchConfiguration('robot_trajectory_ref_topic')),
          ('robot_trajectory_ref_raw', LaunchConfiguration('robot_trajectory_ref_raw_topic')),
        ]
    )


    return LaunchDescription([
        screen_arg,
        ars_path_planner_core_node_name_arg,
        ars_path_planner_manager_node_name_arg,
        config_param_path_planner_core_yaml_file_arg,
        flag_use_state_estim_arg,
        flag_use_slam_map_arg,
        topics_state_estim_funct,
        topics_map_funct,
        robot_pose_des_topic_arg,
        robot_trajectory_ref_topic_arg,
        robot_trajectory_ref_raw_topic_arg,
        GroupAction([
          PushRosNamespace('robot_intelligence'),
          GroupAction([
            PushRosNamespace('path_planner'),
            ars_path_planner_core_node,
            ars_path_planner_manager_node,
          ]),
        ]),
    ])
