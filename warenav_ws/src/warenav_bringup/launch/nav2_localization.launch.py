"""
nav2_localization.launch.py
Navigate using a pre-saved map (AMCL localisation).

Usage:
  ros2 launch warenav_bringup nav2_localization.launch.py map:=/tmp/warenav_map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg   = get_package_share_directory('warenav_description')
    nav_pkg    = get_package_share_directory('warenav_nav')
    gz_ros_pkg = get_package_share_directory('gazebo_ros')
    nav2_pkg   = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map', default_value='/tmp/warenav_map.yaml',
        description='Saved map YAML path')

    urdf_file  = os.path.join(desc_pkg, 'urdf',   'warenav_amr.urdf.xacro')
    world_file = os.path.join(desc_pkg, 'worlds', 'warehouse.world')
    rviz_file  = os.path.join(desc_pkg, 'rviz',   'warenav.rviz')
    nav2_cfg   = os.path.join(nav_pkg,  'config', 'nav2_params.yaml')
    ekf_cfg    = os.path.join(nav_pkg,  'config', 'ekf.yaml')

    robot_desc = Command(['xacro ', urdf_file])

    rsp  = Node(package='robot_state_publisher', executable='robot_state_publisher',
                parameters=[{'robot_description': robot_desc, 'use_sim_time': True}])
    jsp  = Node(package='joint_state_publisher', executable='joint_state_publisher',
                parameters=[{'use_sim_time': True}])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'warenav_amr',
                   '-x', '-9.5', '-y', '-7.2', '-z', '0.08', '-Y', '1.5707'],
        output='screen'
    )
    ekf  = Node(package='robot_localization', executable='ekf_node',
                parameters=[ekf_cfg, {'use_sim_time': True}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'map':          LaunchConfiguration('map'),
            'params_file':  nav2_cfg,
            'autostart':    'true',
        }.items()
    )
    mission = Node(package='warenav_nav', executable='mission_controller.py',
                   parameters=[{'use_sim_time': True, 'skip_exploration': True}],
                   output='screen')
    obstacle = Node(package='warenav_nav', executable='obstacle_detector.py',
                    parameters=[{'use_sim_time': True}], output='screen')
    inv_log  = Node(package='warenav_nav', executable='inventory_logger.py',
                    parameters=[{'use_sim_time': True}], output='screen')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_file] if os.path.exists(rviz_file) else [],
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        map_arg, rsp, jsp, gazebo, spawn,
        TimerAction(period=5.0,  actions=[ekf]),
        TimerAction(period=8.0,  actions=[nav2]),
        TimerAction(period=15.0, actions=[mission, obstacle, inv_log, rviz]),
    ])
