import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = get_package_share_directory('antenna_tracker_simulation')

    xacro_file = os.path.join(pkg_sim, 'urdf', 'antenna_tracker.urdf.xacro')
    world_file = os.path.join(pkg_sim, 'worlds', 'antenna_tracker.world')
    rviz_config = os.path.join(pkg_sim, 'config', 'rviz_config.rviz')

    robot_description = Command(['xacro ', xacro_file])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gz_env = {
        'DISPLAY': os.environ.get('DISPLAY', ':99'),
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'XDG_RUNTIME_DIR': '/tmp/runtime-root',
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Force IGN transport to use loopback (avoids multi-interface confusion in host-network Docker)
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),

        # Gazebo Fortress Server (headless, server-only)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-s', '-r', world_file],
            additional_env=gz_env,
            output='screen'
        ),

        # Spawn URDF in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'antenna_tracker',
                       '-string', robot_description,
                       '-z', '0.1'],
            output='screen'
        ),

        # ROS-GZ Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Gazebo → ROS (sensor data)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/imu/raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
                '/world/antenna_tracker_world/model/antenna_tracker/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                # ROS → Gazebo (joint velocity commands)
                '/model/antenna_tracker/joint/azimuth_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/antenna_tracker/joint/elevation_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            ],
            remappings=[
                ('/world/antenna_tracker_world/model/antenna_tracker/joint_state', '/joint_states'),
            ],
            output='screen'
        ),

        # Simulated target GPS publisher - 풍선 출발지 전남 나주 (ascending phase start)
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--rate', '1',
                '/antenna/target_gps',
                'antenna_tracker_msgs/msg/TargetGPS',
                '{"latitude": 35.0300, "longitude": 126.7100, "altitude_m": 5000.0, "rssi_dbm": -65.0}',
            ],
            output='screen'
        ),

        # Motor velocity translator node
        Node(
            package='antenna_tracker_simulation',
            executable='sim_motor_bridge_node',
            name='sim_motor_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),

        # Sensor Fusion Node
        Node(
            package='antenna_tracker_controller',
            executable='sensor_fusion_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Navigation Node
        Node(
            package='antenna_tracker_controller',
            executable='navigation_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Controller Node (NMPC)
        Node(
            package='antenna_tracker_controller',
            executable='controller_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # State Machine Node
        Node(
            package='antenna_tracker_controller',
            executable='state_machine_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
