"""
navigation.launch.py
--------------------
Starts:
  1. robot_state_publisher    – publishes URDF / static TFs
  2. stage_ros2               – realistic simulation (Scout Mini omni)
  3. nav2 (map_server + amcl + bt_navigator + controller + planner +
            recoveries + lifecycle_manager)
  4. goal_publisher           – reads csv/goals.csv and sends goals via action
  5. rviz2                    – map, tf, particle cloud, laser, paths, goals

Prerequisites:
  • Build and save the map from Task 1 into second_project/map/
      (map.yaml + map.pgm)
  • The Stage world file references second_project/map/map.pgm

Usage:
  ros2 launch second_project navigation.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir      = get_package_share_directory('second_project')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Stage simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'map', 'map.yaml'),
        description='Full path to the map yaml file'
    )
    map_yaml = LaunchConfiguration('map')

    # ── Robot state publisher (URDF → /robot_description + static TFs) ───────
    with open(os.path.join(pkg_dir, 'urdf', 'scout_mini.urdf'), 'r') as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'robot_description':   robot_description,
        }]
    )

    # ── Stage simulation ──────────────────────────────────────────────────────
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        output='screen',
        arguments=[os.path.join(pkg_dir, 'worlds', 'second_project.world')],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            # Stage publishes on /robot_0/cmd_vel by default; remap to /cmd_vel
            ('/robot_0/cmd_vel', '/cmd_vel'),
            ('/robot_0/odom',    '/odom'),
            ('/robot_0/base_scan', '/scan'),
        ]
    )

    # ── Nav2 bringup ──────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map':          map_yaml,
            'params_file':  os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        }.items()
    )

    # ── Goal publisher ────────────────────────────────────────────────────────
    goal_publisher_node = Node(
        package='second_project',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_arg,
        robot_state_publisher_node,
        stage_node,
        nav2_launch,
        goal_publisher_node,
        rviz_node,
    ])
