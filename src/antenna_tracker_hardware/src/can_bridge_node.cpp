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

  /* Receive all relevant IDs — ESP32 (0x100-0x101) + STM32H7 (0x200-0x205) */
  struct can_filter rfilter[8];
  rfilter[0].can_id   = CAN_ID_TARGET_GPS;    rfilter[0].can_mask = CAN_SFF_MASK;
  rfilter[1].can_id   = CAN_ID_TARGET_STATUS;  rfilter[1].can_mask = CAN_SFF_MASK;
  rfilter[2].can_id   = CAN_ID_ACCEL;          rfilter[2].can_mask = CAN_SFF_MASK;
  rfilter[3].can_id   = CAN_ID_GYRO;           rfilter[3].can_mask = CAN_SFF_MASK;
  rfilter[4].can_id   = CAN_ID_MAG;            rfilter[4].can_mask = CAN_SFF_MASK;
  rfilter[5].can_id   = CAN_ID_GPS_FIX;        rfilter[5].can_mask = CAN_SFF_MASK;
  rfilter[6].can_id   = CAN_ID_ENCODER;        rfilter[6].can_mask = CAN_SFF_MASK;
  rfilter[7].can_id   = CAN_ID_HEARTBEAT;      rfilter[7].can_mask = CAN_SFF_MASK;
  setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  RCLCPP_INFO(get_logger(),
    "CAN bridge on %s — RX: ESP32 (0x100-0x101) + STM32H7 (0x200-0x205), TX: 0x300",
    can_iface.c_str());

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

    switch (frame.can_id & CAN_SFF_MASK) {
      /* ESP32 LoRa */
      case CAN_ID_TARGET_GPS:    process_target_gps(frame);    break;
      case CAN_ID_TARGET_STATUS: process_target_status(frame); break;
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
  if (frame.can_dlc < 8) { return; }

  int32_t lat_raw, lon_raw;
  std::memcpy(&lat_raw, &frame.data[0], 4);
  std::memcpy(&lon_raw, &frame.data[4], 4);

  auto msg = antenna_tracker_msgs::msg::TargetGPS();
  msg.header.stamp    = now();
  msg.header.frame_id = "target";
  msg.latitude        = lat_raw / 1e7;
  msg.longitude       = lon_raw / 1e7;
  msg.altitude_m      = 0.0;  /* filled by TARGET_STATUS */
  msg.rssi_dbm        = rssi_dbm_;
  msg.link_quality    = link_quality_;
  pub_target_gps_->publish(msg);
}

void CanBridgeNode::process_target_status(const struct can_frame & frame)
{
  if (frame.can_dlc < 5) { return; }

  int16_t altitude, rssi;
  std::memcpy(&altitude, &frame.data[0], 2);
  std::memcpy(&rssi,     &frame.data[2], 2);

  rssi_dbm_     = static_cast<float>(rssi);
  link_quality_ = frame.data[4];
}

/* ── STM32H7 sensor frame handlers ─────────────────────────────────────── */

/* 0x200: accel (int16 × 1000, m/s²) — buffers until gyro arrives */
void CanBridgeNode::process_accel(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) { return; }

  int16_t ax, ay, az;
  ax = (int16_t)((frame.data[0] << 8) | frame.data[1]);
  ay = (int16_t)((frame.data[2] << 8) | frame.data[3]);
  az = (int16_t)((frame.data[4] << 8) | frame.data[5]);

  pending_imu_.header.stamp               = now();
  pending_imu_.header.frame_id            = "imu_link";
  pending_imu_.linear_acceleration.x     = ax / 1000.0;
  pending_imu_.linear_acceleration.y     = ay / 1000.0;
  pending_imu_.linear_acceleration.z     = az / 1000.0;
  pending_imu_.linear_acceleration_covariance[0] = -1; /* unknown */
  accel_ready_ = true;

  if (gyro_ready_) {
    pub_imu_->publish(pending_imu_);
    accel_ready_ = false;
    gyro_ready_  = false;
  }
}

/* 0x201: gyro (int16 × 1000, rad/s) — publishes combined IMU when both ready */
void CanBridgeNode::process_gyro(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) { return; }

  int16_t gx, gy, gz;
  gx = (int16_t)((frame.data[0] << 8) | frame.data[1]);
  gy = (int16_t)((frame.data[2] << 8) | frame.data[3]);
  gz = (int16_t)((frame.data[4] << 8) | frame.data[5]);

  pending_imu_.angular_velocity.x                = gx / 1000.0;
  pending_imu_.angular_velocity.y                = gy / 1000.0;
  pending_imu_.angular_velocity.z                = gz / 1000.0;
  pending_imu_.angular_velocity_covariance[0]    = -1; /* unknown */
  pending_imu_.orientation_covariance[0]         = -1; /* no orientation */
  gyro_ready_ = true;

  if (accel_ready_) {
    pub_imu_->publish(pending_imu_);
    accel_ready_ = false;
    gyro_ready_  = false;
  }
}

/* 0x202: mag (int16 × 100, µT) */
void CanBridgeNode::process_mag(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) { return; }

  int16_t mx, my, mz;
  mx = (int16_t)((frame.data[0] << 8) | frame.data[1]);
  my = (int16_t)((frame.data[2] << 8) | frame.data[3]);
  mz = (int16_t)((frame.data[4] << 8) | frame.data[5]);

  auto msg = sensor_msgs::msg::MagneticField();
  msg.header.stamp    = now();
  msg.header.frame_id = "imu_link";
  msg.magnetic_field.x = mx / 100.0 * 1e-6; /* µT → T */
  msg.magnetic_field.y = my / 100.0 * 1e-6;
  msg.magnetic_field.z = mz / 100.0 * 1e-6;
  pub_mag_->publish(msg);
}

/* 0x203: GPS fix (lat×1e7, lon×1e7 as int32 LE) */
void CanBridgeNode::process_gps_fix(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }

  int32_t lat_raw, lon_raw;
  std::memcpy(&lat_raw, &frame.data[0], 4);
  std::memcpy(&lon_raw, &frame.data[4], 4);

  auto msg = sensor_msgs::msg::NavSatFix();
  msg.header.stamp    = now();
  msg.header.frame_id = "gps_link";
  msg.latitude        = lat_raw / 1e7;
  msg.longitude       = lon_raw / 1e7;
  msg.status.status   = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  pub_gps_->publish(msg);
}

/* 0x204: encoder (az×10, el×10 as int16 BE + flags uint8) */
void CanBridgeNode::process_encoder(const struct can_frame & frame)
{
  if (frame.can_dlc < 5) { return; }

  int16_t az_raw = (int16_t)((frame.data[0] << 8) | frame.data[1]);
  int16_t el_raw = (int16_t)((frame.data[2] << 8) | frame.data[3]);
  uint8_t flags  = frame.data[4];

  auto msg = antenna_tracker_msgs::msg::EncoderFeedback();
  msg.header.stamp    = now();
  msg.header.frame_id = "base_link";
  msg.az_angle_deg    = az_raw / 10.0;
  msg.el_angle_deg    = el_raw / 10.0;
  msg.az_valid        = (flags & 0x01) != 0;
  msg.el_valid        = (flags & 0x02) != 0;
  pub_encoder_->publish(msg);
}

/* 0x205: heartbeat — log only */
void CanBridgeNode::process_heartbeat(const struct can_frame & frame)
{
  if (frame.can_dlc < 5) { return; }

  uint32_t uptime_ms;
  std::memcpy(&uptime_ms, &frame.data[0], 4);
  uint8_t status = frame.data[4];

  RCLCPP_DEBUG(get_logger(), "STM32H7 heartbeat: uptime=%u ms, status=%u",
               uptime_ms, status);
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

  int16_t az_freq_raw = static_cast<int16_t>(msg->az_frequency_hz * 10.0);
  int16_t el_freq_raw = static_cast<int16_t>(msg->el_frequency_hz * 10.0);
  uint8_t flags = 0;
  if (msg->az_direction)  { flags |= 0x01; }
  if (msg->el_direction)  { flags |= 0x02; }
  if (msg->emergency_stop){ flags |= 0x04; }

  uint8_t data[5];
  std::memcpy(data,     &az_freq_raw, 2);
  std::memcpy(data + 2, &el_freq_raw, 2);
  data[4] = flags;

  send_can_frame(CAN_ID_MOTOR_CMD, data, 5);
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

}  // namespace antenna_tracker_hardware

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_hardware::CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
