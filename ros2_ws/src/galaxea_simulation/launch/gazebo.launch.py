import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('galaxea_simulation')

    urdf_file = os.path.join(pkg, 'urdf', 'r1_v2_1_0.urdf')
    world_file = os.path.join(pkg, 'worlds', 'empty.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'r1',
            '-topic', 'robot_description',
            '-z', '0.05',
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )

    # Bridge Ignition <-> ROS 2
    # If topics don't appear, run: ign topic -l   (inside container)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/default/model/r1/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/r1/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
    ])
