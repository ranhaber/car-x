# Yard maps

This directory holds the saved occupancy grid used for localization and Nav2.

## Producing `yard_map`

1. On the ROCK 4D (ROS 2 Jazzy sourced), with the C1 connected:

   ```bash
   ros2 launch cat_follow_bringup mapping.launch.py
   ```

2. Teleop the car slowly around the full yard perimeter and interior until the
   map in RViz (run on a PC, not the board) is closed and consistent.

3. Save the map into this folder:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f \
     $(ros2 pkg prefix cat_follow_bringup)/share/cat_follow_bringup/maps/yard_map
   ```

   This writes `yard_map.yaml` + `yard_map.pgm`. Copy both back here and commit.

## Origin alignment with the overhead *yard* frame

The overhead camera yard frame (Interface Spec §14) uses **+X right, +Y
forward**; ROS `map` uses REP-103 (**+X forward, +Y left**). Record the
transform between them when you save the map:

- Place the car at the overhead-frame origin, heading along overhead +Y.
- Note the `map`-frame pose reported by slam_toolbox at that instant.
- That pose is the `map -> yard` offset; it is applied in
  `cat_follow/navigation/ros_bridge.py` when populating `NavigationState.heading`.

Document the measured offset (x, y, yaw) alongside the committed map so the
bridge and the overhead pose hints stay consistent.
