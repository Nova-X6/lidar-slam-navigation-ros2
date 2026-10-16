"""
mapping.launch.py
-----------------
Starts:
  1. pointcloud_to_laserscan  – converts /ugv/rslidar_points → /scan
  2. slam_toolbox (async)     – builds a 2-D occupancy map
  3. rviz2                    – visualises map, laser, tf (global frame = map)

Usage:
  ros2 launch second_project mapping.launch.py
  # Play bag in a separate terminal:
  ros2 bag play <bag_file> --clock
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── pointcloud_to_laserscan (built into this package) ────────────────────
    # Converts the 3-D Robosense lidar pointcloud to a 2-D LaserScan
    # so that slam_toolbox can ingest it.
    pointcloud_to_laserscan_node = Node(
        package='second_project',
        executable='pointcloud_to_laserscan',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time':    use_sim_time,
            'min_height':      -0.10,
            'max_height':       0.50,
            'angle_min':       -3.14159265,
            'angle_max':        3.14159265,
            'angle_increment':  0.00872665,
            'scan_time':        0.1,
            'range_min':        0.3,
            'range_max':       30.0,
        }]
    )

    # ── slam_toolbox ──────────────────────────────────────────────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time},
        ]
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'mapping.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        pointcloud_to_laserscan_node,
        slam_toolbox_node,
        rviz_node,
    ])
