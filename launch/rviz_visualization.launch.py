from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    rviz_config_file_arg=DeclareLaunchArgument(
      'rviz_config_file',
      default_value=PathJoinSubstitution(['rviz_conf_sim.rviz']),
      description='Path to the rviz config file'
    )

    #
    rviz_config_file = PathJoinSubstitution([FindPackageShare('ars_robot_intelligence_config'), 'rviz', LaunchConfiguration('rviz_config_file')])


    # Define the nodes
    rviz2_node=Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2_node',
      output=LaunchConfiguration('screen'),
      arguments=['-d', rviz_config_file],
    )

    #
    return LaunchDescription([
      screen_arg,
      rviz_config_file_arg,
      rviz2_node,
    ])
