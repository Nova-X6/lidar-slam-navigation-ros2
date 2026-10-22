# LiDAR Mapping and Autonomous Navigation

Project completed at Politecnico di Milano in May 2026.

A ROS 2 academic robotics project combining LiDAR preprocessing, 2D SLAM, and simulated autonomous waypoint navigation for an AgileX Scout Mini omnidirectional robot.

## Features

- A C++ node converting `/ugv/rslidar_points` (`PointCloud2`) into `/scan` (`LaserScan`), with configurable height, range, and angular filtering.
- SLAM Toolbox configuration for building a 2D occupancy map from recorded LiDAR data.
- Nav2 configuration with AMCL omnidirectional localization, global path planning, local control, and obstacle costmaps.
- A C++ Nav2 action client reading sequential `(x, y, theta)` goals from CSV, reporting feedback, and continuing after rejected, aborted, or canceled goals.
- Stage world, robot URDF, Python launch files, and RViz configurations.

## Technologies

C++, Python launch files, ROS 2, SLAM Toolbox, Nav2, AMCL, Stage ROS 2, RViz2, URDF, YAML, CMake/ament.

## Repository structure

```text
second_project/
  src/       Point-cloud conversion and sequential navigation goal client
  launch/    Mapping and navigation launch files
  config/    SLAM, Nav2, and RViz configurations
  csv/       Navigation goals
  urdf/      Robot description
  worlds/    Stage simulation world
  map/       Instructions and destination for runtime map files
map_folder/  Occupancy map and metadata supplied in the original archive
```

## Build

Use Linux with ROS 2, colcon, SLAM Toolbox, Navigation2, RViz2, and a compatible `stage_ros2` installation. Source your ROS installation first.

```bash
mkdir -p ~/ros2_ws/src
cp -r second_project ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select second_project
source install/setup.bash
```

The original configuration includes ROS-distribution-specific Nav2 plugin names and dependencies. Resolve these for your installed distribution; a clean build across ROS 2 versions has not been verified.

## Mapping from recorded data

Start the mapping pipeline with the recorded-data clock enabled:

```bash
ros2 launch second_project mapping.launch.py use_sim_time:=true
```

In a second sourced terminal:

```bash
ros2 bag play /path/to/lidar_bag --clock
```

The recording must provide `/ugv/rslidar_points` and the transforms/odometry required by SLAM Toolbox. The recorded dataset is not included.

Save the map after reviewing it in RViz:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/second_project/map/map
```

## Simulation and navigation

The supplied map is preserved in `map_folder/`. To use it, copy `map.pgm` and `map.yaml` into `second_project/map/` before rebuilding. Alternatively, generate your own map using the mapping workflow.

Before running navigation:

1. Update `second_project/worlds/second_project.world`: its `40 x 30 m` floorplan is a placeholder. Match the bitmap dimensions and resolution, and align its world pose with the map metadata.
2. Choose a robot start pose in free space and align the AMCL initial pose with that start position.
3. Review `second_project/csv/goals.csv` so its goals lie in the intended map frame.
4. Rebuild and source the workspace to install the updated map and configuration.

```bash
ros2 launch second_project navigation.launch.py
```

The goal client waits for Nav2's `navigate_to_pose` action server and then submits the CSV goals in sequence. Advancing through the list does not mean every goal succeeded; inspect the result logs.

## Validation status

This repository was prepared from the archived academic project. Source and configuration files were inspected, but the ROS 2 pipeline has not been rebuilt or runtime-tested in the Windows publishing environment. The saved map is included as an original artifact; no navigation success rate or mapping accuracy is claimed.

## Academic context

Academic robotics coursework. The repository excludes submission identifiers, course handouts, and recorded datasets.
