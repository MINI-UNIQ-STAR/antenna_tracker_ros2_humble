#ifndef ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_
#define ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <antenna_tracker_msgs/msg/balloon_telemetry.hpp>
#include <antenna_tracker_msgs/msg/encoder_feedback.hpp>
#include <antenna_tracker_msgs/msg/motor_command.hpp>
#include <antenna_tracker_msgs/msg/target_gps.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace antenna_tracker_hardware
{

/**
 * CanBridgeNode — single CAN bus bridge for all three hardware nodes.
 *
 * RX (CAN → ROS 2):
 *   0x100  TARGET_GPS    → /antenna/target_gps         (ESP32 LoRa)
 *   0x101  TARGET_STATUS → (updates rssi/link quality)  (ESP32 LoRa)
 *   0x102  BALLOON_UTC   → /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x103  BALLOON_ACCEL → /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x104  BALLOON_GYROMAG→/antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x105  BALLOON_ORIENT→ /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x106  BALLOON_ENV   → /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x107  BALLOON_PRESS → /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x108  BALLOON_AIR   → /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x109  BALLOON_SYS   → /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x10A  BALLOON_META  → /antenna/balloon_telemetry  (ESP32 LoRa)
 *   0x200  ACCEL         → /imu/raw  (accel part)       (STM32H7)
 *   0x201  GYRO          → /imu/raw  (gyro part)        (STM32H7)
 *   0x202  MAG           → /magnetic_field              (STM32H7)
 *   0x203  GPS           → /gps/fix                    (STM32H7)
 *   0x204  ENCODER       → /antenna/encoder_feedback    (STM32H7)
 *   0x205  HEARTBEAT     → (logged only)                (STM32H7)
 *
 * TX (ROS 2 → CAN):
 *   0x300  MOTOR_CMD     ← /antenna/motor_cmd           (to STM32H7)
 *
 * NOTE: CAN 필터 배열 크기는 17개로 설정해야 합니다.
 *       (기존 8개: 0x100~0x101, 0x200~0x205 + 신규 9개: 0x102~0x10A)
 */
class CanBridgeNode : public rclcpp::Node
{
public:
  explicit CanBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanBridgeNode() override;

private:
  /* ── RX thread & frame dispatch ──────────────────────────────────────── */
  void rx_thread_func();

  /* ESP32 LoRa frames */
  void process_target_gps(const struct can_frame & frame);
  void process_target_status(const struct can_frame & frame);

  /* ESP32 LoRa balloon telemetry frames (0x102~0x10A) */
  void process_balloon_utc(const struct can_frame & frame);
  void process_balloon_accel(const struct can_frame & frame);
  void process_balloon_gyromag(const struct can_frame & frame);
  void process_balloon_orient(const struct can_frame & frame);
  void process_balloon_env(const struct can_frame & frame);
  void process_balloon_press(const struct can_frame & frame);
  void process_balloon_air(const struct can_frame & frame);
  void process_balloon_sys(const struct can_frame & frame);
  void process_balloon_meta(const struct can_frame & frame);
  void try_publish_balloon_telemetry();

  /* STM32H7 sensor frames */
  void process_accel(const struct can_frame & frame);
  void process_gyro(const struct can_frame & frame);
  void process_mag(const struct can_frame & frame);
  void process_gps_fix(const struct can_frame & frame);
  void process_encoder(const struct can_frame & frame);
  void process_heartbeat(const struct can_frame & frame);

  /* TX: /antenna/motor_cmd → CAN 0x300 */
  void motor_cmd_callback(const antenna_tracker_msgs::msg::MotorCommand::SharedPtr msg);
  void send_can_frame(uint32_t id, const uint8_t * data, uint8_t dlc);

  /* STM32H7 heartbeat watchdog (1 Hz timer) */
  void heartbeat_watchdog_callback();

  /* ── CAN ID constants ─────────────────────────────────────────────────── */
  static constexpr uint32_t CAN_ID_TARGET_GPS    = 0x100;
  static constexpr uint32_t CAN_ID_TARGET_STATUS = 0x101;
  // ESP32 LoRa 확장 프레임 (0x102~0x10A)
  static constexpr uint32_t CAN_ID_BALLOON_UTC     = 0x102;
  static constexpr uint32_t CAN_ID_BALLOON_ACCEL   = 0x103;
  static constexpr uint32_t CAN_ID_BALLOON_GYROMAG = 0x104;
  static constexpr uint32_t CAN_ID_BALLOON_ORIENT  = 0x105;
  static constexpr uint32_t CAN_ID_BALLOON_ENV     = 0x106;
  static constexpr uint32_t CAN_ID_BALLOON_PRESS   = 0x107;
  static constexpr uint32_t CAN_ID_BALLOON_AIR     = 0x108;
  static constexpr uint32_t CAN_ID_BALLOON_SYS     = 0x109;
  static constexpr uint32_t CAN_ID_BALLOON_META    = 0x10A;
  static constexpr uint32_t CAN_ID_ACCEL         = 0x200;
  static constexpr uint32_t CAN_ID_GYRO          = 0x201;
  static constexpr uint32_t CAN_ID_MAG           = 0x202;
  static constexpr uint32_t CAN_ID_GPS_FIX       = 0x203;
  static constexpr uint32_t CAN_ID_ENCODER       = 0x204;
  static constexpr uint32_t CAN_ID_HEARTBEAT     = 0x205;
  static constexpr uint32_t CAN_ID_MOTOR_CMD     = 0x300;

  /* ── ROS 2 publishers ─────────────────────────────────────────────────── */
  rclcpp::Publisher<antenna_tracker_msgs::msg::TargetGPS>::SharedPtr    pub_target_gps_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                   pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr         pub_mag_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr             pub_gps_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::EncoderFeedback>::SharedPtr pub_encoder_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::BalloonTelemetry>::SharedPtr pub_balloon_telem_;

  /* ── ROS 2 subscriber ─────────────────────────────────────────────────── */
  rclcpp::Subscription<antenna_tracker_msgs::msg::MotorCommand>::SharedPtr sub_motor_cmd_;

  /* ── Diagnostics timer (1 Hz) ─────────────────────────────────────────── */
  rclcpp::TimerBase::SharedPtr heartbeat_watchdog_timer_;

  /* ── CAN socket ───────────────────────────────────────────────────────── */
  int can_socket_{-1};
  std::thread rx_thread_;
  std::atomic<bool> running_{false};

  /* ── Mutex: protects shared state between RX thread and ROS callbacks ─── */
  std::mutex data_mutex_;

  /* ── Partial IMU assembly (accel + gyro arrive as separate frames) ────── */
  sensor_msgs::msg::Imu pending_imu_;
  bool accel_ready_{false};
  bool gyro_ready_{false};

  /* ── Latest LoRa status fields (filled by 0x101, used in 0x100) ──────── */
  float   rssi_dbm_{0.0f};
  uint8_t link_quality_{0};

  /* ── Balloon telemetry assembly (11 CAN frames → 1 ROS2 msg) ─────────── */
  antenna_tracker_msgs::msg::BalloonTelemetry pending_balloon_;
  uint16_t balloon_rx_mask_{0};           // 비트마스크: 수신된 프레임 추적
  static constexpr uint16_t BALLOON_FULL_MASK = 0x07FFu;  // 0x100~0x10A = 11 bits

  /* ── STM32H7 heartbeat watchdog ───────────────────────────────────────── */
  rclcpp::Time last_heartbeat_time_;
  bool stm32_alive_{false};
};

}  // namespace antenna_tracker_hardware

#endif  // ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_
