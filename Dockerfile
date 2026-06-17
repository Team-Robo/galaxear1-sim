# Use ROS 2 Humble Desktop as the base
FROM osrf/ros:humble-desktop

# Avoid user interaction prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Fortress, ros_gz bridge, Nav2, SLAM Toolbox
RUN apt-get update && apt-get install -y \
    wget \
    nano \
    python3-colcon-common-extensions \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-teleop-twist-keyboard \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory to our mounted workspace
WORKDIR /ros2_ws

# Source the ROS 2 setup file and set standard variables automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN echo "export IGN_GAZEBO_RESOURCE_PATH=/ros2_ws/install/galaxea_simulation/share/galaxea_simulation" >> ~/.bashrc

CMD ["/bin/bash"]