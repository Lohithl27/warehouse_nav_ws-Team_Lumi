"""
slam_mapping.launch.py
Drive robot manually with teleop and build the warehouse map.
Save map when done, then use nav2_localization.launch.py.

Usage:
  ros2 launch warenav_bringup slam_mapping.launch.py
  # Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard
  # Save map:   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/tmp/warenav_map'}}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg   = get_package_share_directory('warenav_description')
    nav_pkg    = get_package_share_directory('warenav_nav')
    gz_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file  = os.path.join(desc_pkg, 'urdf',   'warenav_amr.urdf.xacro')
    world_file = os.path.join(desc_pkg, 'worlds', 'warehouse.world')
    rviz_file  = os.path.join(desc_pkg, 'rviz',   'warenav.rviz')
    slam_cfg   = os.path.join(nav_pkg,  'config', 'slam_toolbox.yaml')
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
                parameters=[ekf_cfg, {'use_sim_time': True}], output='screen')
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                parameters=[slam_cfg, {'use_sim_time': True}], output='screen')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_file] if os.path.exists(rviz_file) else [],
                parameters=[{'use_sim_time': True}])
    teleop = Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
                  output='screen', prefix='xterm -e',
                  remappings=[('/cmd_vel', '/cmd_vel')])

    return LaunchDescription([
        rsp, jsp, gazebo, spawn,
        TimerAction(period=5.0, actions=[ekf, slam]),
        TimerAction(period=8.0, actions=[rviz, teleop]),
    ])
