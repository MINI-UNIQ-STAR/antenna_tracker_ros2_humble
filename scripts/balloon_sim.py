#!/usr/bin/env python3
"""
Balloon trajectory simulator for antenna tracker testing.

Scenario:
  - Ground station : 하동 금오산  35.0650°N 127.7600°E  849 m
  - Launch (나주)  : 35.0300°N 126.7100°E    0 m
  - Burst point    : 35.2620°N 127.2280°E  30000 m  (midpoint, interpolated)
  - Landing (밀양) : 35.4940°N 128.7460°E    0 m

Usage (inside Docker container):
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  export CYCLONEDDS_URI=file:///root/cyclonedds.xml
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  python3 /ros2_ws/scripts/balloon_sim.py [--speed SPEED] [--rate RATE]

  --speed  : simulation speed multiplier (default 60 → 1 real-sec = 1 sim-min)
  --rate   : publish rate in Hz (default 2)

Total sim time at speed=60:
  Ascent  : 30 km / 5 m/s sim = 6000 sim-s → 100 real-s
  Descent : 30 km / 8 m/s sim = 3750 sim-s →  62 real-s
  Total   : ~163 real-seconds (~2.7 minutes)
"""

import time
import argparse

def lerp(a, b, t):
    return a + (b - a) * t

LAUNCH  = (35.0300, 126.7100)   # 나주
BURST   = (35.2620, 127.2280)   # 중간 burst 지점
LANDING = (35.4940, 128.7460)   # 밀양
BURST_ALT = 30000.0             # m
ASCENT_RATE  = 5.0              # m/s (sim)
DESCENT_RATE = 8.0              # m/s (sim)

ASCENT_SIM_SEC  = BURST_ALT / ASCENT_RATE   # 6000 sim-s
DESCENT_SIM_SEC = BURST_ALT / DESCENT_RATE  # 3750 sim-s

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=60.0,
                        help='Sim speed multiplier (default 60)')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Publish rate Hz (default 2)')
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from antenna_tracker_msgs.msg import TargetGPS

    rclpy.init()
    node = Node('balloon_sim')
    pub  = node.create_publisher(TargetGPS, '/antenna/target_gps', 10)

    ascent_real  = ASCENT_SIM_SEC  / args.speed
    descent_real = DESCENT_SIM_SEC / args.speed
    dt = 1.0 / args.rate

    node.get_logger().info(
        f'Balloon sim START  speed={args.speed}x  rate={args.rate} Hz')
    node.get_logger().info(
        f'  Ascent  : {ascent_real:.0f} s real-time')
    node.get_logger().info(
        f'  Descent : {descent_real:.0f} s real-time')
    node.get_logger().info(
        f'  Launch  : {LAUNCH}  Burst: {BURST}@{BURST_ALT}m  Landing: {LANDING}')

    t_start = time.time()
    rssi = -65.0

    try:
        while rclpy.ok():
            elapsed = time.time() - t_start

            if elapsed < ascent_real:
                # ── Ascending ──────────────────────────────
                t = elapsed / ascent_real
                lat = lerp(LAUNCH[0], BURST[0], t)
                lon = lerp(LAUNCH[1], BURST[1], t)
                alt = lerp(0.0, BURST_ALT, t)
                phase = 'ASCENT'
                # RSSI improves as balloon gets higher (closer sight line)
                rssi = lerp(-85.0, -55.0, t)
            elif elapsed < ascent_real + descent_real:
                # ── Descending ─────────────────────────────
                t = (elapsed - ascent_real) / descent_real
                lat = lerp(BURST[0], LANDING[0], t)
                lon = lerp(BURST[1], LANDING[1], t)
                alt = lerp(BURST_ALT, 0.0, t)
                phase = 'DESCENT'
                rssi = lerp(-55.0, -90.0, t)
            else:
                node.get_logger().info('Balloon sim COMPLETE — publishing landing position')
                lat, lon, alt = LANDING[0], LANDING[1], 0.0
                phase = 'LANDED'
                rssi = -95.0

            msg = TargetGPS()
            msg.latitude   = lat
            msg.longitude  = lon
            msg.altitude_m = alt
            msg.rssi_dbm   = rssi
            pub.publish(msg)

            node.get_logger().info(
                f'[{phase}] t={elapsed:.1f}s  lat={lat:.4f} lon={lon:.4f} alt={alt:.0f}m  rssi={rssi:.0f}dBm',
                throttle_duration_sec=5.0)

            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(dt)

            if phase == 'LANDED' and elapsed > ascent_real + descent_real + 10.0:
                break

    except KeyboardInterrupt:
        node.get_logger().info('Balloon sim interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
