import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_DIR / 'launch'


def load_launch_module(name: str):
    spec = importlib.util.spec_from_file_location(
        f'antenna_tracker_bringup_{name}',
        LAUNCH_DIR / f'{name}.launch.py',
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_web_launch_description_contains_bridge_and_http_server():
    module = load_launch_module('web')
    launch_description = module.generate_launch_description()

    assert isinstance(launch_description, LaunchDescription)

    arguments = [entity for entity in launch_description.entities if isinstance(entity, DeclareLaunchArgument)]
    assert [argument.name for argument in arguments] == ['port']

    nodes = [entity for entity in launch_description.entities if isinstance(entity, Node)]
    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('rosbridge_server', 'rosbridge_websocket'),
        ('antenna_tracker_web', 'web_server_node'),
    ]


def test_hardware_launch_description_contains_full_control_stack():
    module = load_launch_module('hardware')
    launch_description = module.generate_launch_description()

    assert isinstance(launch_description, LaunchDescription)

    arguments = [entity for entity in launch_description.entities if isinstance(entity, DeclareLaunchArgument)]
    assert [argument.name for argument in arguments] == ['can_interface']

    nodes = [entity for entity in launch_description.entities if isinstance(entity, Node)]
    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('antenna_tracker_hardware', 'can_bridge_node'),
        ('antenna_tracker_controller', 'sensor_fusion_node'),
        ('antenna_tracker_controller', 'navigation_node'),
        ('antenna_tracker_controller', 'controller_node'),
        ('antenna_tracker_controller', 'state_machine_node'),
    ]


def test_sim_bringup_launch_description_includes_simulation_and_web():
    module = load_launch_module('sim')
    launch_description = module.generate_launch_description()

    assert isinstance(launch_description, LaunchDescription)

    arguments = [entity for entity in launch_description.entities if isinstance(entity, DeclareLaunchArgument)]
    assert [argument.name for argument in arguments] == ['launch_rviz']

    includes = [entity for entity in launch_description.entities if isinstance(entity, IncludeLaunchDescription)]
    assert len(includes) == 2
