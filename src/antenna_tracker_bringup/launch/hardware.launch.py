import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('antenna_tracker_bringup')
    params_file = os.path.join(pkg_bringup, 'config', 'hardware_params.yaml')

    return LaunchDescription([

        # CAN Bridge Node (ESP32 LoRa + STM32H7 sensors via single CAN bus)
        Node(
            package='antenna_tracker_hardware',
            executable='can_bridge_node',
            name='can_bridge_node',
            parameters=[params_file],
            output='screen'
        ),

        # Sensor Fusion Node
        Node(
            package='antenna_tracker_controller',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            parameters=[params_file],
            output='screen'
        ),

        # Navigation Node
        Node(
            package='antenna_tracker_controller',
            executable='navigation_node',
            name='navigation_node',
            parameters=[params_file],
            output='screen'
        ),

        # Controller Node
        Node(
            package='antenna_tracker_controller',
            executable='controller_node',
            name='controller_node',
            parameters=[params_file],
            output='screen'
        ),

        # State Machine Node
        Node(
            package='antenna_tracker_controller',
            executable='state_machine_node',
            name='state_machine_node',
            parameters=[params_file],
            output='screen'
        ),
    ])
