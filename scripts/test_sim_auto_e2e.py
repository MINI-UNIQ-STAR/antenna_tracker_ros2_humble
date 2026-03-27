#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from antenna_tracker_msgs.msg import AntennaState, TargetGPS
from antenna_tracker_msgs.srv import SetMode


class AutoE2EHarness(Node):
    def __init__(self, args):
        super().__init__('auto_e2e_harness')
        self.args = args

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', reliable_qos)
        self.target_pub = self.create_publisher(TargetGPS, '/antenna/target_gps', reliable_qos)
        self.mode_client = self.create_client(SetMode, '/antenna/set_mode')

        self.target_az_sub = self.create_subscription(
            Float64, '/antenna/target_azimuth', self._target_az_callback, reliable_qos)
        self.target_el_sub = self.create_subscription(
            Float64, '/antenna/target_elevation', self._target_el_callback, reliable_qos)
        self.state_sub = self.create_subscription(
            AntennaState, '/antenna/state', self._state_callback, reliable_qos)

        self.last_target_az = None
        self.last_target_el = None
        self.last_state = None

        self.publish_timer = None

    def _target_az_callback(self, msg):
        self.last_target_az = msg.data

    def _target_el_callback(self, msg):
        self.last_target_el = msg.data

    def _state_callback(self, msg):
        self.last_state = msg

    def start_publishers(self):
        if self.publish_timer is None:
            self.publish_timer = self.create_timer(0.1, self._publish_inputs)

    def _publish_inputs(self):
        gps = NavSatFix()
        gps.status.status = 0
        gps.latitude = self.args.ground_lat
        gps.longitude = self.args.ground_lon
        gps.altitude = self.args.ground_alt
        self.gps_pub.publish(gps)

        target = TargetGPS()
        target.latitude = self.args.target_lat
        target.longitude = self.args.target_lon
        target.altitude_m = self.args.target_alt
        self.target_pub.publish(target)

    def is_converged(self):
        if self.last_target_az is None or self.last_target_el is None or self.last_state is None:
            return False
        if not self.last_state.tracking_active:
            return False
        if math.isnan(self.last_state.az_error) or math.isnan(self.last_state.el_error):
            return False
        if abs(self.last_state.target_azimuth) < 0.1 and abs(self.last_state.target_elevation) < 0.1:
            return False
        return (
            abs(self.last_state.az_error) <= self.args.az_tol and
            abs(self.last_state.el_error) <= self.args.el_tol
        )

    def dump_snapshot(self):
        print('=== AUTO E2E snapshot ===')
        print(f'target_azimuth_topic={self.last_target_az}')
        print(f'target_elevation_topic={self.last_target_el}')
        if self.last_state is None:
            print('state=None')
            return
        print(f'current_azimuth={self.last_state.current_azimuth}')
        print(f'current_elevation={self.last_state.current_elevation}')
        print(f'target_azimuth={self.last_state.target_azimuth}')
        print(f'target_elevation={self.last_state.target_elevation}')
        print(f'az_error={self.last_state.az_error}')
        print(f'el_error={self.last_state.el_error}')
        print(f'mode={self.last_state.mode}')
        print(f'tracking_active={self.last_state.tracking_active}')


def spin_until(executor, predicate, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if predicate():
            return True
    executor.spin_once(timeout_sec=0.0)
    return predicate()


def main():
    parser = argparse.ArgumentParser(description='Deterministic AUTO-mode Gazebo E2E harness')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--az-tol', type=float, default=1.0)
    parser.add_argument('--el-tol', type=float, default=1.0)
    parser.add_argument('--ground-lat', type=float, default=35.0650)
    parser.add_argument('--ground-lon', type=float, default=127.7600)
    parser.add_argument('--ground-alt', type=float, default=849.0)
    parser.add_argument('--target-lat', type=float, default=35.0750)
    parser.add_argument('--target-lon', type=float, default=127.7600)
    parser.add_argument('--target-alt', type=float, default=1600.0)
    args = parser.parse_args()

    rclpy.init()
    node = AutoE2EHarness(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        if not spin_until(executor, node.mode_client.service_is_ready, 30.0):
            print('Timed out waiting for /antenna/set_mode service')
            return 1

        request = SetMode.Request()
        request.mode = 0
        future = node.mode_client.call_async(request)
        if not spin_until(executor, lambda: future.done(), 10.0):
            print('Timed out waiting for AUTO mode response')
            return 1

        response = future.result()
        if response is None or not response.success:
            print(f'Failed to switch to AUTO: {response}')
            return 1

        node.start_publishers()
        if not spin_until(executor, node.is_converged, args.timeout):
            print('AUTO E2E did not converge within timeout')
            node.dump_snapshot()
            return 1

        print('=== AUTO E2E converged ===')
        node.dump_snapshot()
        return 0
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
