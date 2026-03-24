"""
simulation.launch.py — WarehouseNav AMR Full Simulation
════════════════════════════════════════════════════════
ONE COMMAND launches the complete stack:
  1. Gazebo + Warehouse World
  2. Robot (URDF + state publisher)
  3. EKF Sensor Fusion
  4. SLAM Toolbox (lifelong async)
  5. Nav2 (MPPI + SmacPlanner)
  6. Frontier Explorer Node
  7. Mission Controller Node
  8. Obstacle Detector Node
  9. Inventory Logger Node
  10. RViz2 Dashboard

Usage:
  ros2 launch warenav_bringup simulation.launch.py
  ros2 launch warenav_bringup simulation.launch.py auto_start:=true
  ros2 launch warenav_bringup simulation.launch.py use_rviz:=false
  ros2 launch warenav_bringup simulation.launch.py skip_exploration:=true
════════════════════════════════════════════════════════
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, LogInfo, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # ── PACKAGE PATHS ─────────────────────────────────
    desc_pkg   = get_package_share_directory('warenav_description')
    nav_pkg    = get_package_share_directory('warenav_nav')
    nav2_pkg   = get_package_share_directory('nav2_bringup')
    gz_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file    = os.path.join(desc_pkg, 'urdf',   'warenav_amr.urdf.xacro')
    world_file   = os.path.join(desc_pkg, 'worlds', 'warehouse.world')
    rviz_file    = os.path.join(desc_pkg, 'rviz',   'warenav.rviz')
    nav2_params  = os.path.join(nav_pkg,  'config', 'nav2_params.yaml')
    slam_params  = os.path.join(nav_pkg,  'config', 'slam_toolbox.yaml')
    ekf_params   = os.path.join(nav_pkg,  'config', 'ekf.yaml')

    # ── LAUNCH ARGUMENTS ──────────────────────────────
    args = [
        DeclareLaunchArgument('use_rviz',         default_value='true'),
        DeclareLaunchArgument('auto_start',        default_value='false'),
        DeclareLaunchArgument('skip_exploration',  default_value='false'),
        DeclareLaunchArgument('use_sim_time',      default_value='true'),
        DeclareLaunchArgument('gz_verbose',        default_value='false'),
    ]

    use_rviz        = LaunchConfiguration('use_rviz')
    auto_start      = LaunchConfiguration('auto_start')
    skip_explore    = LaunchConfiguration('skip_exploration')

    # ── ROBOT DESCRIPTION ─────────────────────────────
    robot_desc = Command(['xacro ', urdf_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # ── GAZEBO ────────────────────────────────────────
    gz_env = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(desc_pkg, 'models') + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world':   world_file,
            'verbose': LaunchConfiguration('gz_verbose'),
            'pause':   'false',
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'warenav_amr',
            '-x', '-9.5', '-y', '-7.2', '-z', '0.08', '-Y', '1.5707',
        ],
        output='screen'
    )

    # ── EKF ───────────────────────────────────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}]
    )

    # ── SLAM TOOLBOX ──────────────────────────────────
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}]
    )

    # ── NAV2 ──────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items()
    )

    # ── WARENAV NODES ─────────────────────────────────
    frontier_explorer = Node(
        package='warenav_nav',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':        True,
            'coverage_threshold':  0.75,
            'min_frontier_size':   8,
            'goal_timeout_sec':    30.0,
            'auto_start':          auto_start,
        }]
    )

    mission_controller = Node(
        package='warenav_nav',
        executable='mission_controller.py',
        name='mission_controller',
        output='screen',
        parameters=[{
            'use_sim_time':        True,
            'auto_start':          auto_start,
            'waypoint_pause_sec':  2.0,
            'max_retries':         3,
            'skip_exploration':    skip_explore,
        }]
    )

    obstacle_detector = Node(
        package='warenav_nav',
        executable='obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
        parameters=[{
            'use_sim_time':      True,
            'emergency_dist_m':  0.4,
            'warning_dist_m':    0.8,
        }]
    )

    inventory_logger = Node(
        package='warenav_nav',
        executable='inventory_logger.py',
        name='inventory_logger',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── RVIZ2 ─────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file] if os.path.exists(rviz_file) else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # ── STARTUP LOG ───────────────────────────────────
    log = LogInfo(msg=(
        '\n' + '═'*58 + '\n'
        'WarehouseNav AMR — Simulation Starting\n'
        'MAHE Mobility Challenge 2026 | Robotics Track\n'
        '═'*58 + '\n'
        'Startup sequence:\n'
        '  t=0s   Gazebo + Robot spawned\n'
        '  t=5s   EKF + SLAM started\n'
        '  t=10s  Nav2 started\n'
        '  t=18s  Mission nodes started\n'
        '  t=20s  RViz2 opened\n'
        '═'*58 + '\n'
        'To start mission:\n'
        '  ros2 service call /start_mission std_srvs/srv/Trigger\n'
        'Direct shelf navigation:\n'
        '  ros2 topic pub --once /nav_cmd std_msgs/msg/String \'{\"data\": \"{\\\"shelf\\\": \\\"B2\\\"}\"}\'\n'
        '═'*58
    ))

    return LaunchDescription([
        *args,
        log,
        gz_env,
        rsp, jsp,
        gazebo,
        spawn_robot,
        TimerAction(period=5.0,  actions=[ekf, slam]),
        TimerAction(period=10.0, actions=[nav2]),
        TimerAction(period=18.0, actions=[
            frontier_explorer,
            mission_controller,
            obstacle_detector,
            inventory_logger,
        ]),
        TimerAction(period=20.0, actions=[rviz]),
    ])
