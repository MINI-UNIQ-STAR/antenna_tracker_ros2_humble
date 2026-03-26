#include "antenna_tracker_hardware/can_bridge_node.hpp"
#include <cstring>

namespace antenna_tracker_hardware
{

// balloon_rx_mask_ 비트 위치 (0x100=bit0 ~ 0x10A=bit10)
static constexpr uint16_t BIT_GPS     = (1 << 0);
static constexpr uint16_t BIT_STATUS  = (1 << 1);
static constexpr uint16_t BIT_UTC     = (1 << 2);
static constexpr uint16_t BIT_ACCEL   = (1 << 3);
static constexpr uint16_t BIT_GYROMAG = (1 << 4);
static constexpr uint16_t BIT_ORIENT  = (1 << 5);
static constexpr uint16_t BIT_ENV     = (1 << 6);
static constexpr uint16_t BIT_PRESS   = (1 << 7);
static constexpr uint16_t BIT_AIR     = (1 << 8);
static constexpr uint16_t BIT_SYS     = (1 << 9);
static constexpr uint16_t BIT_META    = (1 << 10);

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
  // Only publish RSSI if we've received a status frame
  if ((balloon_rx_mask_ & BIT_STATUS) != 0) {
    msg.rssi_dbm     = rssi_dbm_;
    msg.link_quality = link_quality_;
  } else {
    msg.rssi_dbm     = 0.0f;
    msg.link_quality = 0;
  }
  pub_target_gps_->publish(msg);

  // balloon telemetry 조립
  pending_balloon_.latitude  = lat_raw / 1e7;
  pending_balloon_.longitude = lon_raw / 1e7;
  balloon_rx_mask_ |= BIT_GPS;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_target_status(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }

  // altitude in raw int16: unit is METERS (range ±32767 m)
  // Must match ESP32 firmware packing in antenna_tracker_esp32/can_sender.cpp
  int16_t altitude, rssi;
  std::memcpy(&altitude, &frame.data[0], 2);
  std::memcpy(&rssi,     &frame.data[2], 2);

  rssi_dbm_     = static_cast<float>(rssi);
  link_quality_ = frame.data[4];

  // balloon 조립
  pending_balloon_.kf_altitude_m  = static_cast<float>(altitude);
  pending_balloon_.rssi_dbm       = static_cast<int16_t>(rssi);
  pending_balloon_.gps_fix        = frame.data[4];
  pending_balloon_.gps_sats_used  = frame.data[5];
  pending_balloon_.utc_hour       = frame.data[6];
  pending_balloon_.utc_min        = frame.data[7];
  balloon_rx_mask_ |= BIT_STATUS;
  try_publish_balloon_telemetry();
}

/* ── STM32H7 sensor frame handlers ─────────────────────────────────────── */
// CAN frame byte order: big-endian (MSB first) for all sources
// ESP32 LoRa (0x100-0x10A) and STM32H7 (0x200-0x205)
// Exception: process_gps_fix (0x203) uses memcpy — little-endian (native x86/ARM LE)

/* 0x200: accel (int16 × 1000, m/s²) — buffers until gyro arrives */
void CanBridgeNode::process_accel(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) { return; }

  int16_t ax, ay, az;
  ax = static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | static_cast<uint16_t>(frame.data[1]));
  ay = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | static_cast<uint16_t>(frame.data[3]));
  az = static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) | static_cast<uint16_t>(frame.data[5]));

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
  gx = static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | static_cast<uint16_t>(frame.data[1]));
  gy = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | static_cast<uint16_t>(frame.data[3]));
  gz = static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) | static_cast<uint16_t>(frame.data[5]));

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
  mx = static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | static_cast<uint16_t>(frame.data[1]));
  my = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | static_cast<uint16_t>(frame.data[3]));
  mz = static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) | static_cast<uint16_t>(frame.data[5]));

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

  int16_t az_raw = static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | static_cast<uint16_t>(frame.data[1]));
  int16_t el_raw = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | static_cast<uint16_t>(frame.data[3]));
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

/* ── ESP32 Balloon Telemetry frame handlers (0x102~0x10A) ───────────────── */

void CanBridgeNode::process_balloon_utc(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  pending_balloon_.utc_sec          = frame.data[0];
  pending_balloon_.utc_day          = frame.data[1];
  pending_balloon_.utc_month        = frame.data[2];
  pending_balloon_.gps_sats_in_view = frame.data[3];
  uint16_t year, flags;
  std::memcpy(&year,  &frame.data[4], 2);
  std::memcpy(&flags, &frame.data[6], 2);
  pending_balloon_.utc_year     = year;
  pending_balloon_.status_flags = flags;
  balloon_rx_mask_ |= BIT_UTC;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_accel(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  int16_t ax, ay, az, gx;
  std::memcpy(&ax, &frame.data[0], 2);
  std::memcpy(&ay, &frame.data[2], 2);
  std::memcpy(&az, &frame.data[4], 2);
  std::memcpy(&gx, &frame.data[6], 2);
  pending_balloon_.accel_x_mps2 = ax / 100.0f;
  pending_balloon_.accel_y_mps2 = ay / 100.0f;
  pending_balloon_.accel_z_mps2 = az / 100.0f;
  pending_balloon_.gyro_x_rads  = gx / 1000.0f;
  balloon_rx_mask_ |= BIT_ACCEL;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_gyromag(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  int16_t gy, gz, mx, my;
  std::memcpy(&gy, &frame.data[0], 2);
  std::memcpy(&gz, &frame.data[2], 2);
  std::memcpy(&mx, &frame.data[4], 2);
  std::memcpy(&my, &frame.data[6], 2);
  pending_balloon_.gyro_y_rads = gy / 1000.0f;
  pending_balloon_.gyro_z_rads = gz / 1000.0f;
  pending_balloon_.mag_x_ut    = mx / 10.0f;
  pending_balloon_.mag_y_ut    = my / 10.0f;
  balloon_rx_mask_ |= BIT_GYROMAG;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_orient(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  int16_t mz, roll, pitch, palt;
  std::memcpy(&mz,    &frame.data[0], 2);
  std::memcpy(&roll,  &frame.data[2], 2);
  std::memcpy(&pitch, &frame.data[4], 2);
  std::memcpy(&palt,  &frame.data[6], 2);
  pending_balloon_.mag_z_ut         = mz   / 10.0f;
  pending_balloon_.kf_roll_deg      = roll  / 100.0f;
  pending_balloon_.kf_pitch_deg     = pitch / 100.0f;
  pending_balloon_.press_altitude_m = static_cast<float>(palt);
  balloon_rx_mask_ |= BIT_ORIENT;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_env(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  int16_t bt, et, st;
  uint16_t rh;
  std::memcpy(&bt, &frame.data[0], 2);
  std::memcpy(&et, &frame.data[2], 2);
  std::memcpy(&st, &frame.data[4], 2);
  std::memcpy(&rh, &frame.data[6], 2);
  pending_balloon_.board_temp_c     = bt / 100.0f;
  pending_balloon_.external_temp_c  = et / 100.0f;
  pending_balloon_.sht31_temp_c     = st / 100.0f;
  pending_balloon_.sht31_rh_percent = rh / 100.0f;
  balloon_rx_mask_ |= BIT_ENV;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_press(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  uint32_t press;
  int16_t  temp;
  uint16_t co2;
  std::memcpy(&press, &frame.data[0], 4);
  std::memcpy(&temp,  &frame.data[4], 2);
  std::memcpy(&co2,   &frame.data[6], 2);
  pending_balloon_.ms5611_press_pa = press;
  pending_balloon_.ms5611_temp_c   = temp / 100.0f;
  pending_balloon_.co2_ppm         = co2;
  balloon_rx_mask_ |= BIT_PRESS;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_air(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  uint16_t pm1, pm25, pm10;
  int16_t  oz;
  std::memcpy(&pm1,  &frame.data[0], 2);
  std::memcpy(&pm25, &frame.data[2], 2);
  std::memcpy(&pm10, &frame.data[4], 2);
  std::memcpy(&oz,   &frame.data[6], 2);
  pending_balloon_.pm1_ugm3  = pm1;
  pending_balloon_.pm25_ugm3 = pm25;
  pending_balloon_.pm10_ugm3 = pm10;
  pending_balloon_.ozone_ppb = oz;
  balloon_rx_mask_ |= BIT_AIR;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_sys(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) { return; }
  uint16_t gdk, bat;
  int16_t  btemp;
  std::memcpy(&gdk,   &frame.data[0], 2);
  std::memcpy(&bat,   &frame.data[2], 2);
  std::memcpy(&btemp, &frame.data[4], 2);
  pending_balloon_.gdk101_usvh               = gdk   / 100.0f;
  pending_balloon_.bat_mv                    = bat;
  pending_balloon_.bat_temp_c                = btemp / 100.0f;
  pending_balloon_.heater_bat_duty_percent   = frame.data[6];
  pending_balloon_.heater_board_duty_percent = frame.data[7];
  balloon_rx_mask_ |= BIT_SYS;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::process_balloon_meta(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) { return; }
  uint16_t seq;
  uint32_t uptime;
  std::memcpy(&seq,    &frame.data[0], 2);
  std::memcpy(&uptime, &frame.data[2], 4);
  pending_balloon_.seq       = seq;
  pending_balloon_.uptime_ms = uptime;
  balloon_rx_mask_ |= BIT_META;
  try_publish_balloon_telemetry();
}

void CanBridgeNode::try_publish_balloon_telemetry()
{
  if (balloon_rx_mask_ != BALLOON_FULL_MASK) { return; }

  pending_balloon_.header.stamp    = now();
  pending_balloon_.header.frame_id = "balloon";
  pub_balloon_telem_->publish(pending_balloon_);

  balloon_rx_mask_ = 0;
  pending_balloon_ = antenna_tracker_msgs::msg::BalloonTelemetry{};  // 다음 세트를 위해 stale 필드 초기화
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
