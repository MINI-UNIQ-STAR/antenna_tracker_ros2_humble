#ifndef ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_
#define ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

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
 *   0x200  ACCEL         → /imu/raw  (accel part)       (STM32H7)
 *   0x201  GYRO          → /imu/raw  (gyro part)        (STM32H7)
 *   0x202  MAG           → /magnetic_field              (STM32H7)
 *   0x203  GPS           → /gps/fix                    (STM32H7)
 *   0x204  ENCODER       → /antenna/encoder_feedback    (STM32H7)
 *   0x205  HEARTBEAT     → (logged only)                (STM32H7)
 *
 * TX (ROS 2 → CAN):
 *   0x300  MOTOR_CMD     ← /antenna/motor_cmd           (to STM32H7)
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

  /* ── CAN ID constants ─────────────────────────────────────────────────── */
  static constexpr uint32_t CAN_ID_TARGET_GPS    = 0x100;
  static constexpr uint32_t CAN_ID_TARGET_STATUS = 0x101;
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

  /* ── ROS 2 subscriber ─────────────────────────────────────────────────── */
  rclcpp::Subscription<antenna_tracker_msgs::msg::MotorCommand>::SharedPtr sub_motor_cmd_;

  /* ── CAN socket ───────────────────────────────────────────────────────── */
  int can_socket_{-1};
  std::thread rx_thread_;
  std::atomic<bool> running_{false};

  /* ── Partial IMU assembly (accel + gyro arrive as separate frames) ────── */
  sensor_msgs::msg::Imu pending_imu_;
  bool accel_ready_{false};
  bool gyro_ready_{false};

  /* ── Latest LoRa status fields (filled by 0x101, used in 0x100) ──────── */
  float   rssi_dbm_{0.0f};
  uint8_t link_quality_{0};
};

}  // namespace antenna_tracker_hardware

#endif  // ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_
