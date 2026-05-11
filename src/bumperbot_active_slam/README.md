# bumperbot_active_slam

Active SLAM version 1 for Bumper-Bot ROS 2.

This package is a fresh ROS 2/rclpy implementation for the existing Bumper-Bot stack. It is inspired by the path-entropy exploration flow from `MF-Ahmed/aslam_rosbot`, but it does not port the ROS 1 package, Open Karto, g2o, `move_base`, ROS 1 launch files, D-optimality, or pose graph uncertainty code.

## What The Node Does

`active_slam_node` runs this loop:

```text
/map + TF map->base_footprint
    -> frontier detection on OccupancyGrid
    -> frontier clustering
    -> path entropy along robot-to-frontier line
    -> distance penalty and frontier size bonus
    -> best goal selection
    -> NavigateToPose goal for Nav2
```

The map is expected to come from SLAM Toolbox, and navigation is delegated to Nav2 through `/navigate_to_pose`.

## Topics

Subscribed:

- `/map` (`nav_msgs/msg/OccupancyGrid`)

Published:

- `/active_slam/frontiers` (`visualization_msgs/msg/MarkerArray`)
- `/active_slam/selected_goal` (`geometry_msgs/msg/PoseStamped`)
- `/active_slam/status` (`std_msgs/msg/String`)

Action client:

- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)

## Run

Build from the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Two-terminal flow:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py use_sim_time:=true
```

Integrated simulation launch:

```bash
ros2 launch bumperbot_bringup active_slam_simulated_robot.launch.py
```

## RViz Debug

Add these displays:

- `MarkerArray`: `/active_slam/frontiers`
- `PoseStamped`: `/active_slam/selected_goal`

Check runtime status:

```bash
ros2 topic echo /active_slam/status
```

## Important Parameters

- `map_topic`: defaults to `/map`
- `global_frame`: defaults to `map`
- `robot_base_frame`: defaults to `base_footprint`; use `base_link` if your TF tree uses that frame
- `nav2_action_name`: defaults to `/navigate_to_pose`
- `min_frontier_cells`: filters small frontier clusters
- `goal_clearance_radius_m`: rejects goals too close to occupied cells
- `goal_backoff_m`: pulls frontier goals slightly back toward the robot
- `lambda_distance`: controls distance penalty strength
- `entropy_weight`, `frontier_size_weight`: utility score weights
- `blacklist_duration_sec`: temporary blacklist time for failed goals
