import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = get_package_share_directory('antenna_tracker_simulation')

    xacro_file = os.path.join(pkg_sim, 'urdf', 'antenna_tracker.urdf.xacro')
    world_file = os.path.join(pkg_sim, 'worlds', 'antenna_tracker.world')
    rviz_config = os.path.join(pkg_sim, 'config', 'rviz_config.rviz')
    sim_params_file = os.path.join(pkg_sim, 'config', 'sim_params.yaml')

    robot_description = Command(['xacro ', xacro_file])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    encoder_noise_deg = LaunchConfiguration('encoder_noise_deg', default='0.0')
    velocity_noise_dps = LaunchConfiguration('velocity_noise_dps', default='0.0')
    command_noise_hz = LaunchConfiguration('command_noise_hz', default='0.0')
    disturbance_dps2 = LaunchConfiguration('disturbance_dps2', default='0.25')

    gz_env = {
        'DISPLAY': os.environ.get('DISPLAY', ':99'),
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'XDG_RUNTIME_DIR': '/tmp/runtime-root',
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('encoder_noise_deg', default_value='0.0'),
        DeclareLaunchArgument('velocity_noise_dps', default_value='0.0'),
        DeclareLaunchArgument('command_noise_hz', default_value='0.0'),
        DeclareLaunchArgument('disturbance_dps2', default_value='0.25'),

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

        # NOTE: /antenna/target_gps는 balloon_sim.py가 담당 (동적 궤적 + Friis RSSI)
        # docker exec -it antenna_tracker_dev bash -c "... python3 /ros2_ws/scripts/balloon_sim.py --speed 60"

        # Motor velocity translator node
        Node(
            package='antenna_tracker_simulation',
            executable='sim_motor_bridge_node',
            name='sim_motor_bridge',
            parameters=[sim_params_file, {
                'use_sim_time': use_sim_time,
                'encoder_noise_deg': encoder_noise_deg,
                'velocity_noise_dps': velocity_noise_dps,
                'command_noise_hz': command_noise_hz,
                'disturbance_dps2': disturbance_dps2,
            }],
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
            parameters=[sim_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Navigation Node
        Node(
            package='antenna_tracker_controller',
            executable='navigation_node',
            parameters=[sim_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Controller Node (NMPC)
        Node(
            package='antenna_tracker_controller',
            executable='controller_node',
            parameters=[sim_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # State Machine Node
        Node(
            package='antenna_tracker_controller',
            executable='state_machine_node',
            parameters=[sim_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(launch_rviz)
        ),
    ])
