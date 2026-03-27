#include "antenna_tracker_hardware/can_bridge_node.hpp"
#include <cstring>

namespace antenna_tracker_hardware
{

CanBridgeNode::CanBridgeNode(const rclcpp::NodeOptions & options)
: Node("can_bridge_node", options)
{
  declare_parameter<std::string>("can_interface", "can0");
  declare_parameter<int>("can_bitrate", 500000);

  /* ── ROS 2 Publishers ───────────────────────────────────────────────── */
  pub_target_gps_ = create_publisher<antenna_tracker_msgs::msg::TargetGPS>(
    "/antenna/target_gps", rclcpp::SensorDataQoS());

  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
    "/imu/raw", rclcpp::SensorDataQoS());

  pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>(
    "/magnetic_field", rclcpp::SensorDataQoS());

  pub_gps_ = create_publisher<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", rclcpp::SensorDataQoS());

  pub_encoder_ = create_publisher<antenna_tracker_msgs::msg::EncoderFeedback>(
    "/antenna/encoder_feedback", rclcpp::SensorDataQoS());

  pub_balloon_telem_ = create_publisher<antenna_tracker_msgs::msg::BalloonTelemetry>(
    "/balloon/telemetry", rclcpp::SensorDataQoS());

  /* ── ROS 2 Subscriber: motor_cmd → CAN 0x300 ───────────────────────── */
  sub_motor_cmd_ = create_subscription<antenna_tracker_msgs::msg::MotorCommand>(
    "/antenna/motor_cmd", rclcpp::SensorDataQoS(),
    std::bind(&CanBridgeNode::motor_cmd_callback, this, std::placeholders::_1));

  /* ── SocketCAN setup ────────────────────────────────────────────────── */
  std::string can_iface = get_parameter("can_interface").as_string();

  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN socket: %s", strerror(errno));
    return;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(get_logger(), "CAN interface '%s' not found", can_iface.c_str());
    close(can_socket_);
    can_socket_ = -1;
    return;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket");
    close(can_socket_);
    can_socket_ = -1;
    return;
  }

  /* Receive all relevant IDs — ESP32 (0x100-0x10A) + STM32H7 (0x200-0x205) */
  struct can_filter rfilter[17];
  rfilter[0].can_id   = CAN_ID_TARGET_GPS;    rfilter[0].can_mask = CAN_SFF_MASK;
  rfilter[1].can_id   = CAN_ID_TARGET_STATUS;  rfilter[1].can_mask = CAN_SFF_MASK;
  rfilter[2].can_id   = CAN_ID_ACCEL;          rfilter[2].can_mask = CAN_SFF_MASK;
  rfilter[3].can_id   = CAN_ID_GYRO;           rfilter[3].can_mask = CAN_SFF_MASK;
  rfilter[4].can_id   = CAN_ID_MAG;            rfilter[4].can_mask = CAN_SFF_MASK;
  rfilter[5].can_id   = CAN_ID_GPS_FIX;        rfilter[5].can_mask = CAN_SFF_MASK;
  rfilter[6].can_id   = CAN_ID_ENCODER;        rfilter[6].can_mask = CAN_SFF_MASK;
  rfilter[7].can_id   = CAN_ID_HEARTBEAT;      rfilter[7].can_mask = CAN_SFF_MASK;
  rfilter[8].can_id  = 0x102; rfilter[8].can_mask  = CAN_SFF_MASK;
  rfilter[9].can_id  = 0x103; rfilter[9].can_mask  = CAN_SFF_MASK;
  rfilter[10].can_id = 0x104; rfilter[10].can_mask = CAN_SFF_MASK;
  rfilter[11].can_id = 0x105; rfilter[11].can_mask = CAN_SFF_MASK;
  rfilter[12].can_id = 0x106; rfilter[12].can_mask = CAN_SFF_MASK;
  rfilter[13].can_id = 0x107; rfilter[13].can_mask = CAN_SFF_MASK;
  rfilter[14].can_id = 0x108; rfilter[14].can_mask = CAN_SFF_MASK;
  rfilter[15].can_id = 0x109; rfilter[15].can_mask = CAN_SFF_MASK;
  rfilter[16].can_id = 0x10A; rfilter[16].can_mask = CAN_SFF_MASK;
  if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to set CAN RX filter: %s", strerror(errno));
  }

  RCLCPP_INFO(get_logger(),
    "CAN bridge on %s — RX: ESP32 (0x100-0x10A) + STM32H7 (0x200-0x205), TX: 0x300",
    can_iface.c_str());

  /* ── STM32H7 heartbeat watchdog timer (1 Hz) ────────────────────────── */
  last_heartbeat_time_ = now();
  heartbeat_watchdog_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&CanBridgeNode::heartbeat_watchdog_callback, this));

  running_ = true;
  rx_thread_ = std::thread(&CanBridgeNode::rx_thread_func, this);
}

CanBridgeNode::~CanBridgeNode()
{
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (can_socket_ >= 0) {
    close(can_socket_);
  }
}

/* ── RX Thread ──────────────────────────────────────────────────────────── */
void CanBridgeNode::rx_thread_func()
{
  struct can_frame frame;

  while (running_) {
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(can_socket_, &rdfs);

    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 100000; /* 100 ms timeout for graceful shutdown */

    if (select(can_socket_ + 1, &rdfs, nullptr, nullptr, &tv) <= 0) {
      continue;
    }

    ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
      continue;
    }

    /* No outer lock here — each process_* function is called only from this
     * thread, so their internal state (pending_imu_, pending_balloon_, etc.)
     * needs no protection.  process_heartbeat() acquires data_mutex_ itself
     * to protect the cross-thread last_heartbeat_time_ / stm32_alive_ fields. */
    switch (frame.can_id & CAN_SFF_MASK) {
      /* ESP32 LoRa */
      case CAN_ID_TARGET_GPS:    process_target_gps(frame);    break;
      case CAN_ID_TARGET_STATUS: process_target_status(frame); break;
      case 0x102: process_balloon_utc(frame);     break;
      case 0x103: process_balloon_accel(frame);   break;
      case 0x104: process_balloon_gyromag(frame); break;
      case 0x105: process_balloon_orient(frame);  break;
      case 0x106: process_balloon_env(frame);     break;
      case 0x107: process_balloon_press(frame);   break;
      case 0x108: process_balloon_air(frame);     break;
      case 0x109: process_balloon_sys(frame);     break;
      case 0x10A: process_balloon_meta(frame);    break;
      /* STM32H7 sensors */
      case CAN_ID_ACCEL:         process_accel(frame);         break;
      case CAN_ID_GYRO:          process_gyro(frame);          break;
      case CAN_ID_MAG:           process_mag(frame);           break;
      case CAN_ID_GPS_FIX:       process_gps_fix(frame);       break;
      case CAN_ID_ENCODER:       process_encoder(frame);       break;
      case CAN_ID_HEARTBEAT:     process_heartbeat(frame);     break;
      default: break;
    }
  }
}

/* ── ESP32 LoRa frame handlers ──────────────────────────────────────────── */
void CanBridgeNode::process_target_gps(const struct can_frame & frame)
{
  auto msg = balloon_assembler_.process_target_gps(frame);
  if (msg) {
    msg->header.stamp = now();
    msg->header.frame_id = "target";
    pub_target_gps_->publish(*msg);
  }
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_target_status(const struct can_frame & frame)
{
  balloon_assembler_.process_target_status(frame);
  try_publish_balloon_telemetry();
}

/* ── STM32H7 sensor frame handlers ─────────────────────────────────────── */
// CAN frame byte order: big-endian (MSB first) for all sources
// ESP32 LoRa (0x100-0x10A) and STM32H7 (0x200-0x205)
// Exception: process_gps_fix (0x203) uses memcpy — little-endian (native x86/ARM LE)

/* 0x200: accel (int16 × 1000, m/s²) — buffers until gyro arrives */
void CanBridgeNode::process_accel(const struct can_frame & frame)
{
  auto msg = process_accel_frame(frame, imu_state_);
  if (msg) {
    msg->header.stamp = now();
    msg->header.frame_id = "imu_link";
    pub_imu_->publish(*msg);
  }
}

/* 0x201: gyro (int16 × 1000, rad/s) — publishes combined IMU when both ready */
void CanBridgeNode::process_gyro(const struct can_frame & frame)
{
  auto msg = process_gyro_frame(frame, imu_state_);
  if (msg) {
    msg->header.stamp = now();
    msg->header.frame_id = "imu_link";
    pub_imu_->publish(*msg);
  }
}

/* 0x202: mag (int16 × 100, µT) */
void CanBridgeNode::process_mag(const struct can_frame & frame)
{
  auto msg = decode_mag_frame(frame);
  if (!msg) {
    return;
  }

  msg->header.stamp = now();
  msg->header.frame_id = "imu_link";
  pub_mag_->publish(*msg);
}

/* 0x203: GPS fix (lat×1e7, lon×1e7 as int32 LE) */
void CanBridgeNode::process_gps_fix(const struct can_frame & frame)
{
  auto msg = decode_gps_fix_frame(frame);
  if (!msg) {
    return;
  }

  msg->header.stamp = now();
  msg->header.frame_id = "gps_link";
  pub_gps_->publish(*msg);
}

/* 0x204: encoder (az×10, el×10 as int16 BE + flags uint8) */
void CanBridgeNode::process_encoder(const struct can_frame & frame)
{
  auto msg = decode_encoder_frame(frame);
  if (!msg) {
    return;
  }

  msg->header.stamp = now();
  msg->header.frame_id = "base_link";
  pub_encoder_->publish(*msg);
}

/* 0x205: heartbeat — update watchdog state */
void CanBridgeNode::process_heartbeat(const struct can_frame & frame)
{
  if (frame.can_dlc < 5) { return; }

  /* Use big-endian decode (MSB first) consistent with all other STM32H7 frames */
  uint32_t uptime_ms = (static_cast<uint32_t>(frame.data[0]) << 24) |
                       (static_cast<uint32_t>(frame.data[1]) << 16) |
                       (static_cast<uint32_t>(frame.data[2]) << 8) |
                       static_cast<uint32_t>(frame.data[3]);
  uint8_t status = frame.data[4];

  /* Narrow lock: only cross-thread fields need protection */
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_heartbeat_time_ = now();
    stm32_alive_ = true;
  }

  RCLCPP_DEBUG(get_logger(), "STM32H7 heartbeat: uptime=%u ms, status=%u",
               uptime_ms, status);
}

/* 1 Hz watchdog: warn if no heartbeat received within 2 s */
void CanBridgeNode::heartbeat_watchdog_callback()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if ((now() - last_heartbeat_time_).seconds() > 2.0) {
    if (stm32_alive_) {
      RCLCPP_WARN(get_logger(), "STM32H7 heartbeat lost — no heartbeat for >2 s");
      stm32_alive_ = false;
    }
  }
}

/* ── TX: /antenna/motor_cmd → CAN 0x300 ────────────────────────────────── */
/*
 * Frame layout (5 bytes):
 *   [0-1] az_freq × 10  (int16 LE)
 *   [2-3] el_freq × 10  (int16 LE)
 *   [4]   flags: bit0=az_dir, bit1=el_dir, bit2=estop
 */
void CanBridgeNode::motor_cmd_callback(
  const antenna_tracker_msgs::msg::MotorCommand::SharedPtr msg)
{
  if (can_socket_ < 0) { return; }

  const auto data = encode_motor_command_payload(*msg);
  send_can_frame(CAN_ID_MOTOR_CMD, data.data(), static_cast<uint8_t>(data.size()));
}

void CanBridgeNode::send_can_frame(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id  = id;
  frame.can_dlc = dlc;
  std::memcpy(frame.data, data, dlc);

  if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
    RCLCPP_WARN(get_logger(), "CAN TX failed for ID 0x%03X: %s", id, strerror(errno));
  }
}

/* ── ESP32 Balloon Telemetry frame handlers (0x102~0x10A) ───────────────── */

void CanBridgeNode::process_balloon_utc(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_utc(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_accel(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_accel(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_gyromag(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_gyromag(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_orient(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_orient(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_env(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_env(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_press(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_press(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_air(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_air(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_sys(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_sys(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_meta(const struct can_frame & frame)
{
  balloon_assembler_.process_balloon_meta(frame);
  try_publish_balloon_telemetry();
}

void CanBridgeNode::try_publish_balloon_telemetry()
{
  auto msg = balloon_assembler_.take_complete_message();
  if (!msg) {
    return;
  }

  msg->header.stamp = now();
  msg->header.frame_id = "balloon";
  pub_balloon_telem_->publish(*msg);
}

}  // namespace antenna_tracker_hardware

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_hardware::CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
