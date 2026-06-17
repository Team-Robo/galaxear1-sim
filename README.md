# RoboCup Navigation - Galaxea R1

Simulation and navigation stack for the Galaxea R1 robot using ROS 2 Humble and Gazebo Fortress.

---

## I. Requirements

| Category | Requirement |
|----------|-------------|
| OS | Ubuntu 22.04 (native or WSL2 on Windows 11) |
| ROS | ROS 2 Humble Desktop |
| Simulator | Gazebo Fortress (via `ros-humble-ros-gz`) |
| GPU | NVIDIA GPU recommended (for Gazebo rendering and `gpu_lidar` sensor) |
| Docker (optional) | Docker Engine + Docker Compose + NVIDIA Container Toolkit |

### ROS / Apt Packages

Installed automatically by Docker or `setup.bash`:

- `ros-humble-ros-gz`, `ros-humble-ros-gz-sim`, `ros-humble-ros-gz-bridge`, `ros-humble-ros-gz-image`
- `ros-humble-navigation2`, `ros-humble-nav2-bringup`
- `ros-humble-slam-toolbox`
- `ros-humble-teleop-twist-keyboard`
- `ros-humble-foxglove-bridge`
- `ros-humble-rqt`, `ros-humble-rqt-common-plugins`
- `python3-colcon-common-extensions`

---

## II. Setup & Installation

### Option A: Docker (Recommended)

**Prerequisites:** Docker, Docker Compose, NVIDIA Container Toolkit, WSLg (if on Windows).

```bash
# Clone the repository
git clone <repo-url> ~/robocup_nav
cd ~/robocup_nav

# Allow GUI forwarding (run once per reboot)
xhost +local:root

# Build and start the container
docker compose build
docker compose up -d

# Enter the container
docker exec -it robocup_nav bash

# Inside the container — build the workspace
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> **Workspace mounting:** Only `./ros2_ws/src` is bind-mounted to `/ros2_ws/src`
> inside the container. Edits from your host editor are reflected instantly.
> `build/`, `install/`, and `log/` live inside the container only, so they
> never pollute the host with root-owned files.

### Option B: Native Ubuntu

**Prerequisites:** Ubuntu 22.04 with ROS 2 Humble Desktop already installed.

```bash
git clone <repo-url> ~/robocup_nav
cd ~/robocup_nav
bash setup.bash
```

`setup.bash` runs `rosdep install` (reads dependencies from `package.xml`), installs colcon, and builds the workspace.

After setup, add to your `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/robocup_nav/ros2_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=~/robocup_nav/ros2_ws/install/galaxea_simulation/share/galaxea_simulation
```

---

## III. How to Run

### Launch the simulation

```bash
# If using Docker, enter the container first:
docker exec -it robocup_nav bash

# Launch Gazebo Fortress with the R1 model
ros2 launch galaxea_simulation gazebo.launch.py
```

This starts Gazebo with an empty world, spawns the R1, publishes TFs, and bridges
`/clock` and `/joint_states` between Ignition and ROS 2.

### Verify topics

```bash
# ROS 2 topics
ros2 topic list

# Ignition topics (useful for debugging bridge issues)
ign topic -l
```

### Stop

```bash
# Ctrl+C the launch, then (if Docker):
docker compose down
```

---

## IV. Repository Structure

```
robocup_nav/
├── Dockerfile                      # Docker image: ROS 2 Humble + Fortress + Nav2
├── docker-compose.yml              # Container config (GPU, display, workspace mount)
├── setup.bash                      # Native Ubuntu setup script
├── README.md
└── ros2_ws/
    └── src/
        └── galaxea_simulation/     # Main simulation package
            ├── CMakeLists.txt
            ├── package.xml         # ROS dependencies (used by rosdep)
            ├── urdf/
            │   └── r1_v2_1_0.urdf  # R1 robot model (SolidWorks export + Gazebo plugins)
            ├── meshes/             # STL meshes for all R1 links
            │   ├── base_link.STL
            │   ├── wheel_motor_link[1-3].STL
            │   ├── steer_motor_link[1-3].STL
            │   ├── torso_link[1-4].STL
            │   ├── left_arm_*.STL
            │   ├── right_arm_*.STL
            │   └── zed_link.STL
            ├── launch/
            │   └── gazebo.launch.py  # Launches Fortress + spawns R1 + bridge
            ├── worlds/
            │   └── empty.sdf         # Fortress world (physics, sensors, ground plane)
            └── config/
                └── joint_names_r1_v2_1_0.yaml
```

---

## V. Technical Explanation

### Robot Model

The Galaxea R1 URDF (34 links, 33 joints) was exported from SolidWorks. The base is a
3-wheel omnidirectional/swerve drive (3 steer revolute joints + 3 continuous wheel joints
in a triangle layout). The upper body has a torso, dual 6-DOF arms with grippers, and
camera mounts (ZED stereo + RealSense).

### Gazebo Fortress Integration

Gazebo Classic plugins do not work with Fortress. The URDF includes Fortress-specific
`<gazebo>` extensions:

| Plugin | Purpose |
|--------|---------|
| `ignition::gazebo::systems::JointStatePublisher` | Publishes joint states for TF computation |
| `gpu_lidar` sensor on `lidar_link` | 360-degree lidar, 12m range, publishes to `/scan` (Ignition topic) |
| `imu` sensor on `imu_link` | 100Hz IMU, publishes to `/imu` (Ignition topic) |

> **Note:** Lidar and IMU positions are placeholders (15cm and 5cm above `base_link`).
> Adjust the `<origin>` in `lidar_joint` and `imu_joint` once actual sensor
> mounting locations are confirmed.

### Launch Architecture

```
gazebo.launch.py
├── gz_sim.launch.py          # Starts Gazebo Fortress with empty.sdf world
├── ros_gz_sim/create          # Spawns R1 URDF into the running Gazebo instance
├── robot_state_publisher      # Publishes /tf from URDF + /joint_states
└── ros_gz_bridge              # Bridges Ignition <-> ROS 2 topics:
    ├── /clock                    (Ignition -> ROS)
    └── /joint_states             (Ignition -> ROS)
```

### World SDF

`empty.sdf` loads the required Fortress system plugins at the world level:

- **Physics** — ODE physics engine at 1ms step
- **Sensors** — Renders gpu_lidar via ogre2
- **Imu** — Processes IMU sensor data
- **SceneBroadcaster / UserCommands** — Required for Gazebo GUI and model spawning

### Control (Not Yet Implemented)

The R1's swerve drive requires a controller that maps `cmd_vel` (linear.x, linear.y,
angular.z) to individual steer angles and wheel velocities. Current plan is to use
DiffDrive as a simplified approximation for Nav2 testing (forward/back/turn only,
no lateral movement). Full omnidirectional control can be added later via
`ign_ros2_control` or a custom twist-to-joints node.
