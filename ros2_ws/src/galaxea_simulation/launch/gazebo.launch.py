import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('galaxea_simulation')
    gz_resource_path = os.path.dirname(pkg)

    # r1_sensors.urdf.xacro includes r1_v2_1_0.urdf (the CAD-exported, regenerate-on
    # re-export file — never hand-edit it) and adds the lidar + chassis IMU on top, so
    # sensor additions survive the next SolidWorks re-export.
    urdf_file = os.path.join(pkg, 'urdf', 'r1_sensors.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'apartment.sdf')

    robot_desc = xacro.process_file(urdf_file).toxml()

    set_gz_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path,
    )

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
            '-x', '3',
            '-y', '-3',
            '-z', '0.5',
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
            '/model/r1/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
            '/world/default/model/r1/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Swerve steering commands (ROS -> IGN)
            '/model/r1/joint/steer_motor_joint1/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/r1/joint/steer_motor_joint2/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/r1/joint/steer_motor_joint3/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            # Swerve wheel velocity commands (ROS -> IGN)
            '/model/r1/joint/wheel_motor_joint1/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/r1/joint/wheel_motor_joint2/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/r1/joint/wheel_motor_joint3/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/r1/sensor/livox_lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/model/r1/sensor/chassis_imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        remappings=[
            ('/world/default/model/r1/joint_state', '/joint_states'),
            ('/model/r1/sensor/livox_lidar/points', '/livox/lidar'),
            ('/model/r1/sensor/chassis_imu', '/_gz/imu_chassis_raw'),
        ],
        output='screen',
    )

    swerve_controller = Node(
        package='galaxea_simulation',
        executable='swerve_controller.py',
        name='swerve_controller',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    swerve_odometry = Node(
        package='galaxea_simulation',
        executable='swerve_odometry.py',
        name='swerve_odometry',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    imu_hdas_translator = Node(
        package='galaxea_simulation',
        executable='imu_hdas_translator.py',
        name='imu_hdas_translator',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    livox_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_alias',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'livox_frame',
            '--child-frame-id', 'r1/livox_frame/livox_lidar',
        ],
    )

    return LaunchDescription([
        set_gz_resource,
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        swerve_controller,
        swerve_odometry,
        imu_hdas_translator,
        livox_frame_alias,
    ])
