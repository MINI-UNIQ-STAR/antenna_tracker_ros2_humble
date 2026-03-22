#!/usr/bin/env python3
"""
Balloon trajectory simulator for antenna tracker testing.

Scenario:
  - Ground station : 하동 금오산  35.0650°N 127.7600°E  849 m
  - Launch (나주)  : 35.0300°N 126.7100°E    0 m
  - Burst point    : 35.2620°N 127.2280°E  30000 m  (midpoint, interpolated)
  - Landing (밀양) : 35.4940°N 128.7460°E    0 m

RF Link Budget (LoRa 915 MHz, Friis FSPL):
  - Tx power    : 200 mW  = +23 dBm
  - Tx antenna  : 2 dBi   (balloon)
  - EIRP        : +25 dBm
  - FSPL (dB)   : 32.45 + 20·log10(915) + 20·log10(d_km)
                = 61.68 + 20·log10(d_km)
  - Rx antenna  : 0 dBi   (ground station, isotropic reference)
  - Pr (dBm)    = 25 - 61.68 - 20·log10(d_km)
                = -36.68 - 20·log10(d_km)
  - LoRa SF12/BW125 sensitivity ≈ -137 dBm → link closes up to ~800 km LOS

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
  Descent : 30 km / 6 m/s sim = 5000 sim-s →  83 real-s
  Total   : ~183 real-seconds (~3.1 minutes)
"""

import math
import time
import argparse

def lerp(a, b, t):
    return a + (b - a) * t

# ── Ground station ──────────────────────────────────────────────────────────
GS_LAT = 35.0650   # 하동 금오산
GS_LON = 127.7600
GS_ALT = 849.0     # m

# ── Trajectory waypoints ───────────────────────────────────────────────────
LAUNCH  = (35.0300, 126.7100)   # 나주
BURST   = (35.2620, 127.2280)   # 중간 burst 지점
LANDING = (35.4940, 128.7460)   # 밀양
BURST_ALT = 30000.0             # m
ASCENT_RATE  = 5.0              # m/s
DESCENT_RATE = 6.0              # m/s

# ── RF parameters (LoRa 915 MHz, Friis FSPL) ───────────────────────────────
TX_POWER_DBM   = 23.0   # 200 mW
TX_GAIN_DBI    = 2.0    # balloon antenna
RX_GAIN_DBI    = 0.0    # ground station (isotropic reference)
FREQ_MHZ       = 915.0
FSPL_CONST_DB  = 32.45 + 20 * math.log10(FREQ_MHZ)   # 61.68 dB
EIRP_DBM       = TX_POWER_DBM + TX_GAIN_DBI            # 25 dBm


def haversine_m(lat1, lon1, lat2, lon2):
    """Horizontal great-circle distance in metres."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2)**2
    return R * 2 * math.asin(math.sqrt(a))


def rssi_dbm(lat, lon, alt_m):
    """Compute received power (dBm) via Friis free-space path loss."""
    d_horiz = haversine_m(GS_LAT, GS_LON, lat, lon)
    d_vert  = alt_m - GS_ALT
    d_total = math.sqrt(d_horiz**2 + d_vert**2)
    d_total = max(d_total, 1.0)   # avoid log(0)
    d_km = d_total / 1000.0
    fspl_db = FSPL_CONST_DB + 20 * math.log10(d_km)
    pr = EIRP_DBM + RX_GAIN_DBI - fspl_db
    return pr

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
        f'  Ascent  : {ascent_real:.0f} s real-time  ({ASCENT_RATE} m/s)')
    node.get_logger().info(
        f'  Descent : {descent_real:.0f} s real-time  ({DESCENT_RATE} m/s)')
    node.get_logger().info(
        f'  Launch  : {LAUNCH}  Burst: {BURST}@{BURST_ALT}m  Landing: {LANDING}')
    node.get_logger().info(
        f'  RF: LoRa 915 MHz  Tx={TX_POWER_DBM:.0f}dBm  Ant={TX_GAIN_DBI:.0f}dBi  EIRP={EIRP_DBM:.0f}dBm')

    t_start = time.time()

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
            elif elapsed < ascent_real + descent_real:
                # ── Descending ─────────────────────────────
                t = (elapsed - ascent_real) / descent_real
                lat = lerp(BURST[0], LANDING[0], t)
                lon = lerp(BURST[1], LANDING[1], t)
                alt = lerp(BURST_ALT, 0.0, t)
                phase = 'DESCENT'
            else:
                node.get_logger().info('Balloon sim COMPLETE — publishing landing position')
                lat, lon, alt = LANDING[0], LANDING[1], 0.0
                phase = 'LANDED'

            # Physics-based RSSI (Friis free-space path loss, LoRa 915 MHz)
            rx = rssi_dbm(lat, lon, alt)

            msg = TargetGPS()
            msg.latitude   = lat
            msg.longitude  = lon
            msg.altitude_m = alt
            msg.rssi_dbm   = rx
            pub.publish(msg)

            node.get_logger().info(
                f'[{phase}] t={elapsed:.1f}s  lat={lat:.4f} lon={lon:.4f} '
                f'alt={alt:.0f}m  rssi={rx:.1f}dBm',
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
