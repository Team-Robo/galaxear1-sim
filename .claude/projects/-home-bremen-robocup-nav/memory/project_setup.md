---
name: project-setup
description: RoboCup nav project runs ROS2 Humble in Docker on WSL2 with Gazebo, Nav2, SLAM Toolbox, Foxglove
metadata:
  type: project
---

Docker-based ROS2 Humble development environment for robocup navigation. Stack: Gazebo Classic, Nav2, SLAM Toolbox, TurtleBot3 sim, Foxglove Bridge, teleop-twist-keyboard. NVIDIA GPU passthrough enabled. Workspace at ./ros2_ws mounted into container.

**Why:** Building and testing a navigation stack (mapping, localization, path planning) for RoboCup in a simulated apartment environment.

**How to apply:** All ROS2 commands run inside the Docker container. Prefer Foxglove over RViz for visualization when possible (lighter on WSL2). Custom packages go in ros2_ws/src/.
