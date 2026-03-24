"""
display.launch.py
Visualise the robot in RViz2 without Gazebo.
Use this to verify the URDF is correct before full simulation.

Usage:
  ros2 launch warenav_description display.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('warenav_description')
    urdf = os.path.join(pkg, 'urdf', 'warenav_amr.urdf.xacro')
    rviz = os.path.join(pkg, 'rviz', 'urdf_view.rviz')

    robot_desc = Command(['xacro ', urdf])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen'
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz] if os.path.exists(rviz) else [],
        output='screen'
    )

    return LaunchDescription([rsp, jsp_gui, rviz_node])
