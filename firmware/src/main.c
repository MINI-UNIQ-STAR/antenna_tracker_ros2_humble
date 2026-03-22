#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>

#include <antenna_tracker_msgs/msg/encoder_feedback.h>
#include <antenna_tracker_msgs/msg/heartbeat.h>
#include <antenna_tracker_msgs/msg/motor_command.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/nav_sat_fix.h>

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printk("Failed status on line %d: %d. Aborting.\n", __LINE__,            \
             (int)temp_rc);                                                    \
      return 1;                                                                \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printk("Failed status on line %d: %d. Continuing.\n", __LINE__,          \
             (int)temp_rc);                                                    \
    }                                                                          \
  }

extern bool zephyr_transport_open(struct uxrCustomTransport *transport);
extern bool zephyr_transport_close(struct uxrCustomTransport *transport);
extern size_t zephyr_transport_write(struct uxrCustomTransport *transport,
                                     const uint8_t *buf, size_t len,
                                     uint8_t *err);
extern size_t zephyr_transport_read(struct uxrCustomTransport *transport,
                                    uint8_t *buf, size_t len, int timeout,
                                    uint8_t *err);

rcl_publisher_t encoder_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t mag_pub;
rcl_publisher_t gps_pub;
rcl_publisher_t heartbeat_pub;
rcl_subscription_t motor_sub;

antenna_tracker_msgs__msg__EncoderFeedback encoder_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
sensor_msgs__msg__NavSatFix gps_msg;
antenna_tracker_msgs__msg__MotorCommand motor_msg;
antenna_tracker_msgs__msg__Heartbeat heartbeat_msg;

/* Hardware Peripherals */
static const struct gpio_dt_spec az_step =
    GPIO_DT_SPEC_GET(DT_NODELABEL(az_step), gpios);
static const struct gpio_dt_spec az_dir =
    GPIO_DT_SPEC_GET(DT_NODELABEL(az_dir), gpios);
static const struct gpio_dt_spec az_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(az_en), gpios);

static const struct gpio_dt_spec el_step =
    GPIO_DT_SPEC_GET(DT_NODELABEL(el_step), gpios);
static const struct gpio_dt_spec el_dir =
    GPIO_DT_SPEC_GET(DT_NODELABEL(el_dir), gpios);
static const struct gpio_dt_spec el_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(el_en), gpios);

const struct device *i2c_dev1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
const struct device *i2c_dev2 = DEVICE_DT_GET(DT_NODELABEL(i2c2));

/* GPS UART (Define 'gps_uart' alias in app.overlay) */
#define GPS_UART_NODE DT_ALIAS(gps_uart)
#if DT_NODE_EXISTS(GPS_UART_NODE)
const struct device *uart_dev = DEVICE_DT_GET(GPS_UART_NODE);
#else
const struct device *uart_dev = NULL;
#endif

#define AS5600_ADDR 0x36
#define BMI270_ADDR 0x68
#define MLX90393_ADDR 0x18

/* GPS UART Buffer */
static char gps_rx_buf[128];
static int gps_rx_idx = 0;
static bool gps_line_ready = false;

/* Global Motor Tracking Variables */
float current_az_freq = 0.0;
float current_el_freq = 0.0;

void motor_cmd_callback(const void *msgin) {
  const antenna_tracker_msgs__msg__MotorCommand *msg =
      (const antenna_tracker_msgs__msg__MotorCommand *)msgin;

  if (msg->emergency_stop) {
    gpio_pin_set_dt(&az_en, 0); // Disable motor (Assuming Active Low)
    gpio_pin_set_dt(&el_en, 0);
    current_az_freq = 0.0;
    current_el_freq = 0.0;
    return;
  }

  gpio_pin_set_dt(&az_en, 1);
  gpio_pin_set_dt(&el_en, 1);

  gpio_pin_set_dt(&az_dir, msg->az_direction ? 1 : 0);
  gpio_pin_set_dt(&el_dir, msg->el_direction ? 1 : 0);

  current_az_freq = msg->az_frequency_hz;
  current_el_freq = msg->el_frequency_hz;
}

/* K_Timer for Stepper motor stepping (~10kHz logic) */
void stepper_timer_isr(struct k_timer *dummy) {
  static float az_accum = 0.0;
  static float el_accum = 0.0;
  float dt = 0.0001f; // 100 us

  az_accum += current_az_freq * dt;
  if (az_accum >= 1.0) {
    gpio_pin_toggle_dt(&az_step);
    az_accum -= 1.0;
  }

  el_accum += current_el_freq * dt;
  if (el_accum >= 1.0) {
    gpio_pin_toggle_dt(&el_step);
    el_accum -= 1.0;
  }
}
K_TIMER_DEFINE(stepper_timer, stepper_timer_isr, NULL);

/* BMI270 Registers */
#define BMI270_CHIP_ID_REG 0x00
#define BMI270_ACC_X_LSB 0x0C
#define BMI270_GYR_X_LSB 0x12
#define BMI270_PWR_CONF 0x7C
#define BMI270_PWR_CTRL 0x7D
#define BMI270_CMD_REG 0x7E

/* MLX90393 Commands */
#define MLX_CMD_SB 0x3E   // Single Measurement XYZ
#define MLX_CMD_RM 0x4E   // Read Measurement XYZ
#define MLX_CMD_EXIT 0x80 // Exit Sleep

/* Sensor read functions */
int init_bmi270() {
  uint8_t chip_id = 0;
  uint8_t reg = BMI270_CHIP_ID_REG;
  if (i2c_write_read(i2c_dev1, BMI270_ADDR, &reg, 1, &chip_id, 1) != 0)
    return -1;
  if (chip_id != 0x24)
    return -1;

  uint8_t pwr_up[2] = {BMI270_PWR_CONF, 0x00};
  i2c_write(i2c_dev1, pwr_up, 2, BMI270_ADDR);
  k_msleep(1);
  uint8_t pwr_ctrl[2] = {BMI270_PWR_CTRL, 0x0E}; // Accel + Gyro + Temp
  i2c_write(i2c_dev1, pwr_ctrl, 2, BMI270_ADDR);
  k_msleep(2);
  return 0;
}

int init_mlx90393() {
  uint8_t cmd = MLX_CMD_EXIT;
  return i2c_write(i2c_dev1, &cmd, 1, MLX90393_ADDR);
}

float read_as5600_angle(const struct device *dev) {
  if (!device_is_ready(dev))
    return -1.0;

  uint8_t reg = 0x0E; // RAW ANGLE register
  uint8_t buf[2] = {0};
  if (i2c_write_read(dev, AS5600_ADDR, &reg, 1, buf, 2) == 0) {
    uint16_t raw_angle = (buf[0] << 8) | buf[1];
    return ((float)raw_angle / 4096.0f) * 360.0f;
  }
  return -1.0;
}

void read_bmi270(sensor_msgs__msg__Imu *msg) {
  uint8_t reg = BMI270_ACC_X_LSB;
  uint8_t buf[12]; // 6 bytes Accel + 6 bytes Gyro
  if (i2c_write_read(i2c_dev1, BMI270_ADDR, &reg, 1, buf, 12) == 0) {
    int16_t ax = (buf[1] << 8) | buf[0];
    int16_t ay = (buf[3] << 8) | buf[2];
    int16_t az = (buf[5] << 8) | buf[4];
    msg->linear_acceleration.x =
        (double)ax * (9.80665 / 16384.0); // 16-bit, +/-2g
    msg->linear_acceleration.y = (double)ay * (9.80665 / 16384.0);
    msg->linear_acceleration.z = (double)az * (9.80665 / 16384.0);

    int16_t gx = (buf[7] << 8) | buf[6];
    int16_t gy = (buf[9] << 8) | buf[8];
    int16_t gz = (buf[11] << 8) | buf[10];
    msg->angular_velocity.x =
        (double)gx * (3.14159265 / (180.0 * 16.384)); // 16-bit, +/-2000dps
    msg->angular_velocity.y = (double)gy * (3.14159265 / (180.0 * 16.384));
    msg->angular_velocity.z = (double)gz * (3.14159265 / (180.0 * 16.384));
  }
  /* NOTE: BMI270 requires a configuration blob upload for advanced
     features/calibration. If readings remain zero or inaccurate, implement the
     full init sequence. */
}

void read_mlx90393(sensor_msgs__msg__MagneticField *msg) {
  uint8_t sb_cmd = MLX_CMD_SB;
  i2c_write(i2c_dev1, &sb_cmd, 1, MLX90393_ADDR);
  k_msleep(10); // Conversion time

  uint8_t rm_cmd = MLX_CMD_RM;
  uint8_t buf[7]; // Status + 6 bytes (X, Y, Z)
  if (i2c_write_read(i2c_dev1, MLX90393_ADDR, &rm_cmd, 1, buf, 7) == 0) {
    int16_t mx = (buf[1] << 8) | buf[2];
    int16_t my = (buf[3] << 8) | buf[4];
    int16_t mz = (buf[5] << 8) | buf[6];
    msg->magnetic_field.x = (double)mx * 1e-6; // Simplified scale
    msg->magnetic_field.y = (double)my * 1e-6;
    msg->magnetic_field.z = (double)mz * 1e-6;
  }
}

/* UART ISR Callback for GPS */
void uart_cb(const struct device *dev, void *user_data) {
  uint8_t c;
  if (!uart_irq_update(dev)) return;
  while (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) {
    if (uart_fifo_read(dev, &c, 1) == 1) {
      if (c == '\n' || c == '\r') {
        if (gps_rx_idx > 0) {
          gps_rx_buf[gps_rx_idx] = '\0';
          gps_line_ready = true;
        }
      } else if (gps_rx_idx < sizeof(gps_rx_buf) - 1) {
        gps_rx_buf[gps_rx_idx++] = c;
      }
    }
  }
}

void parse_nmea(const char *line, sensor_msgs__msg__NavSatFix *msg) {
  if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
    msg->status.status = 0; // Fix
    msg->latitude = 37.5665;  // Default for testing
    msg->longitude = 126.9780;
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    // Read AS5600 Encoders
    float az_angle = read_as5600_angle(i2c_dev1);
    if (az_angle >= 0.0) {
      encoder_msg.az_angle_deg = az_angle;
      encoder_msg.az_valid = true;
    } else {
      encoder_msg.az_valid = false;
    }

    float el_angle = read_as5600_angle(i2c_dev2);
    if (el_angle >= 0.0) {
      encoder_msg.el_angle_deg = el_angle;
      encoder_msg.el_valid = true;
    } else {
      encoder_msg.el_valid = false;
    }
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));

    // Read IMU & Mag
    uint32_t ms = k_uptime_get();
    imu_msg.header.stamp.sec = ms / 1000;
    imu_msg.header.stamp.nanosec = (ms % 1000) * 1000000;
    mag_msg.header.stamp = imu_msg.header.stamp;

    read_bmi270(&imu_msg);
    read_mlx90393(&mag_msg);

    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&mag_pub, &mag_msg, NULL));

    if (gps_line_ready) {
      parse_nmea(gps_rx_buf, &gps_msg);
      gps_rx_idx = 0;
      gps_line_ready = false;
      RCSOFTCHECK(rcl_publish(&gps_pub, &gps_msg, NULL));
    }
  }
}

static void init_hardware() {
  gpio_pin_configure_dt(&az_step, GPIO_OUTPUT_INACTIVE);
  gpio_pin_configure_dt(&az_dir, GPIO_OUTPUT_INACTIVE);
  gpio_pin_set_dt(&az_en, 0); // Active Low Disable by default

  gpio_pin_configure_dt(&el_step, GPIO_OUTPUT_INACTIVE);
  gpio_pin_configure_dt(&el_dir, GPIO_OUTPUT_INACTIVE);
  gpio_pin_set_dt(&el_en, 0);

  /* Initialize Sensors */
  if (!device_is_ready(i2c_dev1))
    printk("I2C1 fail\n");
  if (!device_is_ready(i2c_dev2))
    printk("I2C2 fail\n");

  if (init_bmi270() == 0 && init_mlx90393() == 0) {
    printk("BMI270 + MLX90393 Initialized.\n");
  } else {
    printk("Sensor Initialization Failed!\n");
  }

  /* Initialize UART GPS */
  if (device_is_ready(uart_dev)) {
    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    printk("GPS UART Initialized.\n");
  } else {
    printk("GPS UART fail\n");
  }

  k_timer_start(&stepper_timer, K_USEC(100), K_USEC(100)); // 10kHz timer
}

int main(void) {
  printk("Starting Micro-ROS Antenna Tracker Firmware with Hardware "
         "Interface...\n");
  init_hardware();

  rmw_uros_set_custom_transport(true, NULL, zephyr_transport_open,
                                zephyr_transport_close, zephyr_transport_write,
                                zephyr_transport_read);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    k_msleep(1000);
  }

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "tracker_firmware", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &encoder_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(antenna_tracker_msgs, msg, EncoderFeedback),
      "/antenna/encoder_feedback"));

  RCCHECK(rclc_publisher_init_default(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/raw"));

  RCCHECK(rclc_publisher_init_default(
      &mag_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "/magnetic_field"));

  RCCHECK(rclc_publisher_init_default(
      &gps_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
      "/gps/fix"));

  RCCHECK(rclc_publisher_init_default(
      &heartbeat_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(antenna_tracker_msgs, msg, Heartbeat),
      "/antenna/heartbeat"));

  RCCHECK(rclc_subscription_init_default(
      &motor_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(antenna_tracker_msgs, msg, MotorCommand),
      "/antenna/motor_cmd"));

  rcl_timer_t timer;
  RCCHECK(rclc_timer_init_default2(&timer, &support,
                                   RCL_MS_TO_NS(10), // 100Hz
                                   timer_callback, true));

  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg,
                                         &motor_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  uint64_t last_hb_publish = 0;

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    uint64_t now_ms = k_uptime_get();
    if (now_ms - last_hb_publish >= 1000) {
      heartbeat_msg.stamp_ms = (uint32_t)now_ms;
      heartbeat_msg.status = 1; // OK
      RCSOFTCHECK(rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL));
      last_hb_publish = now_ms;
    }

    k_usleep(1000);
  }

  return 0;
}
