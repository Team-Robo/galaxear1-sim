# Galaxea R1 simulation

Simulation for the Galaxea R1 robot using ROS 2 Humble and Gazebo Fortress.

---

## I. Requirements

| Category | Requirement |
|----------|-------------|
| OS | Ubuntu 22.04 (native or WSL2 on Windows 11) |
| ROS | ROS 2 Humble Desktop |
| Simulator | Gazebo Fortress (via `ros-humble-ros-gz`) |
| GPU | NVIDIA GPU recommended (for Gazebo rendering) |
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

### Option A: Docker

**Prerequisites:** Docker, Docker Compose, NVIDIA Container Toolkit, WSLg (if on Windows).

```bash
# Clone the repository
git clone https://github.com/Team-Robo/galaxear1-sim.git ~/galaxear1-sim
cd ~/galaxear1-sim

# Allow GUI forwarding (run once per reboot)
xhost +local:root

# Build and start the container
docker compose build
docker compose up -d

# Enter the container
docker exec -it galaxear1-sim bash

# Inside the container — build the workspace
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```
#### To stop the container
```bash
docker compose down
```
**Workspace mounting:** `./ros2_ws/src` is bind-mounted to `/ros2_ws/src` inside the container. Edits from your host editor are reflected instantly.

**No NVIDIA GPU?** The GPU passthrough lines in `docker-compose.yml` (the two `NVIDIA_*` environment variables and the `deploy` block) are commented out by default, so `docker compose build`/`up` works as-is on any machine — Gazebo just falls back to software (llvmpipe) rendering. If you do have an NVIDIA GPU + the NVIDIA Container Toolkit installed, uncomment those lines for hardware-accelerated rendering.

### Option B: Native Ubuntu

**Prerequisites:** Ubuntu 22.04 with ROS 2 Humble Desktop 

```bash
git clone https://github.com/Team-Robo/galaxear1-sim.git ~/galaxear1-sim
cd ~/galaxear1-sim
bash setup.bash
```

`setup.bash` runs `rosdep install` (reads dependencies from `package.xml`), installs colcon, and builds the workspace.

After setup, add to your `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/galaxear1-sim/ros2_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=~/galaxear1-sim/ros2_ws/install/galaxea_simulation/share
```

---

## III. How to Run

### Launch the simulation

```bash
ros2 launch galaxea_simulation gazebo.launch.py
```
This spawns the R1 and bridges all necessary topics between Ignition and ROS 2.

### Drive the robot with teleop

In a second terminal (inside the container if using Docker):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard to send velocity commands. The swerve controller accepts full
holonomic movement — `linear.x` (forward/back), `linear.y` (strafe), and
`angular.z` (rotate).

### Verify topics

```bash
# ROS 2 topics
ros2 topic list

# Ignition topics (useful for debugging bridge issues)
ign topic -l
```

## IV. Technical Explanation

### Swerve Drive Controller

`swerve_controller.py` is a ROS 2 node that converts `/cmd_vel` (Twist) into individual
joint commands for the 3 swerve modules:

1. Subscribes to `/cmd_vel` (linear.x, linear.y, angular.z)
2. Computes per-wheel velocity vectors using swerve inverse kinematics
3. Optimizes wheel direction (flips steer angle + reverses wheel if delta > 90 degrees)
4. Publishes steer angle commands (`Float64` → `JointPositionController` in Gazebo)
5. Publishes wheel velocity commands (`Float64` → `JointController` in Gazebo)

### Spawn offset vs. odometry

`gazebo.launch.py` spawns `r1` at `(3, -3, 0.5)` in the Gazebo world frame (see the
`spawn` node's `-x`/`-y`/`-z` arguments). `/odom` (and any other pose estimate that
initializes at the origin, e.g. from `swerve_odometry.py`) starts at `(0, 0, 0)` with
yaw `0`, so it will be offset from the raw Gazebo world-frame pose
(`/model/r1/pose`) by that spawn transform for the entire run. Subtract the spawn
pose before comparing odometry against ground truth, or you'll see a constant
`(3, -3)` position error (plus any spawn yaw) that looks like drift but isn't.

