import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'sim.launch.py'


def load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'antenna_tracker_simulation_sim_launch',
        LAUNCH_FILE,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_sim_launch_description_contains_gazebo_bridge_and_control_nodes():
    module = load_launch_module()
    launch_description = module.generate_launch_description()

    assert isinstance(launch_description, LaunchDescription)

    arguments = [entity for entity in launch_description.entities if isinstance(entity, DeclareLaunchArgument)]
    assert [argument.name for argument in arguments] == [
        'use_sim_time',
        'launch_rviz',
        'encoder_noise_deg',
        'velocity_noise_dps',
        'command_noise_hz',
        'disturbance_dps2',
    ]

    assert sum(isinstance(entity, SetEnvironmentVariable) for entity in launch_description.entities) == 1
    assert sum(type(entity) is ExecuteProcess for entity in launch_description.entities) == 1

    nodes = [entity for entity in launch_description.entities if isinstance(entity, Node)]
    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('ros_gz_sim', 'create'),
        ('ros_gz_bridge', 'parameter_bridge'),
        ('antenna_tracker_simulation', 'sim_motor_bridge_node'),
        ('robot_state_publisher', 'robot_state_publisher'),
        ('antenna_tracker_controller', 'sensor_fusion_node'),
        ('antenna_tracker_controller', 'navigation_node'),
        ('antenna_tracker_controller', 'controller_node'),
        ('antenna_tracker_controller', 'state_machine_node'),
        ('rviz2', 'rviz2'),
    ]
