import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = get_package_share_directory('antenna_tracker_simulation')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_sim, 'urdf', 'antenna_tracker.urdf.xacro')
    world_file = os.path.join(pkg_sim, 'worlds', 'antenna_tracker.world')
    rviz_config = os.path.join(pkg_sim, 'config', 'rviz_config.rviz')

    robot_description = Command(['xacro ', xacro_file])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Gazebo Fortress Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r ', world_file]}.items(),
        ),

        # Spawn URDF in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'antenna_tracker',
                       '-topic', 'robot_description',
                       '-z', '0.1'],
            output='screen'
        ),

        # ROS-GZ Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/imu/raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/world/antenna_tracker_world/model/antenna_tracker/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[
                ('/world/antenna_tracker_world/model/antenna_tracker/joint_state', '/joint_states'),
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
