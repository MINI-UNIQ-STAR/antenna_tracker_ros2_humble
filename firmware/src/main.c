#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <string.h>

/* ── CAN Protocol IDs ─────────────────────────────────────────────────────── */
/* TX: STM32H7 → RPi4B (via shared CAN bus) */
#define CAN_ID_ACCEL     0x200  /* ax,ay,az (int16 × 1000, m/s²), 6 bytes */
#define CAN_ID_GYRO      0x201  /* gx,gy,gz (int16 × 1000, rad/s), 6 bytes */
#define CAN_ID_MAG       0x202  /* mx,my,mz (int16 × 100, µT), 6 bytes     */
#define CAN_ID_GPS       0x203  /* lat×1e7, lon×1e7 (int32 LE), 8 bytes    */
#define CAN_ID_ENCODER   0x204  /* az×10,el×10 (int16 LE) + flags, 5 bytes */
#define CAN_ID_HEARTBEAT 0x205  /* uptime_ms (uint32 LE) + status, 5 bytes */

/* RX: RPi4B → STM32H7 */
#define CAN_ID_MOTOR_CMD 0x300  /* az_freq×10,el_freq×10 (int16 LE) + flags (uint8), 5 bytes */
                                /* flags: bit0=az_dir, bit1=el_dir, bit2=estop */

/* ── Hardware Peripherals ─────────────────────────────────────────────────── */
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

#define GPS_UART_NODE DT_ALIAS(gps_uart)
#if DT_NODE_EXISTS(GPS_UART_NODE)
static const struct device *uart_dev = DEVICE_DT_GET(GPS_UART_NODE);
#else
static const struct device *uart_dev = NULL;
#endif

const struct device *can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

/* ── Sensor Addresses ─────────────────────────────────────────────────────── */
#define AS5600_ADDR    0x36
#define BMI270_ADDR    0x68
#define MLX90393_ADDR  0x0C

/* BMI270 Registers */
#define BMI270_CHIP_ID_REG 0x00
#define BMI270_ACC_X_LSB   0x0C
#define BMI270_PWR_CONF    0x7C
#define BMI270_PWR_CTRL    0x7D

/* MLX90393 Commands */
#define MLX_CMD_SB   0x3E
#define MLX_CMD_RM   0x4E
#define MLX_CMD_EXIT 0x80

/* ── GPS UART Buffer ──────────────────────────────────────────────────────── */
static char gps_rx_buf[128];
static int  gps_rx_idx = 0;
static bool gps_line_ready = false;
static double gps_lat = 37.5665;
static double gps_lon = 126.9780;

/* ── Motor State ──────────────────────────────────────────────────────────── */
static volatile float current_az_freq = 0.0f;
static volatile float current_el_freq = 0.0f;
static volatile bool  motor_estop = false;

/* ── Stepper Timer (10kHz) ────────────────────────────────────────────────── */
static void stepper_timer_isr(struct k_timer *dummy)
{
    static float az_accum = 0.0f;
    static float el_accum = 0.0f;
    const float dt = 0.0001f; /* 100 µs */

    if (motor_estop) {
        return;
    }

    az_accum += current_az_freq * dt;
    if (az_accum >= 1.0f) {
        gpio_pin_toggle_dt(&az_step);
        az_accum -= 1.0f;
    }

    el_accum += current_el_freq * dt;
    if (el_accum >= 1.0f) {
        gpio_pin_toggle_dt(&el_step);
        el_accum -= 1.0f;
    }
}
K_TIMER_DEFINE(stepper_timer, stepper_timer_isr, NULL);

/* ── CAN Motor CMD Message Queue ─────────────────────────────────────────── */
CAN_MSGQ_DEFINE(motor_msgq, 4);

/* ── Sensor Functions ─────────────────────────────────────────────────────── */
static int init_bmi270(void)
{
    uint8_t chip_id = 0;
    uint8_t reg = BMI270_CHIP_ID_REG;
    if (i2c_write_read(i2c_dev1, BMI270_ADDR, &reg, 1, &chip_id, 1) != 0) {
        return -1;
    }
    if (chip_id != 0x24) {
        return -1;
    }
    uint8_t pwr_up[2] = {BMI270_PWR_CONF, 0x00};
    i2c_write(i2c_dev1, pwr_up, 2, BMI270_ADDR);
    k_msleep(1);
    uint8_t pwr_ctrl[2] = {BMI270_PWR_CTRL, 0x0E}; /* Accel + Gyro + Temp */
    i2c_write(i2c_dev1, pwr_ctrl, 2, BMI270_ADDR);
    k_msleep(2);
    return 0;
}

static int init_mlx90393(void)
{
    uint8_t cmd = MLX_CMD_EXIT;
    return i2c_write(i2c_dev2, &cmd, 1, MLX90393_ADDR);
}

static float read_as5600_angle(const struct device *dev)
{
    if (!device_is_ready(dev)) {
        return -1.0f;
    }
    uint8_t reg = 0x0E; /* RAW ANGLE register */
    uint8_t buf[2] = {0};
    if (i2c_write_read(dev, AS5600_ADDR, &reg, 1, buf, 2) == 0) {
        uint16_t raw = (buf[0] << 8) | buf[1];
        return ((float)raw / 4096.0f) * 360.0f;
    }
    return -1.0f;
}

static void read_bmi270(int16_t *ax, int16_t *ay, int16_t *az,
                         int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t reg = BMI270_ACC_X_LSB;
    uint8_t buf[12];
    if (i2c_write_read(i2c_dev1, BMI270_ADDR, &reg, 1, buf, 12) != 0) {
        return;
    }
    int16_t raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_az = (int16_t)((buf[5] << 8) | buf[4]);
    /* m/s² × 1000: +/-2g range → scale = 9.80665/16384 × 1000 */
    *ax = (int16_t)((float)raw_ax * (9.80665f / 16384.0f) * 1000.0f);
    *ay = (int16_t)((float)raw_ay * (9.80665f / 16384.0f) * 1000.0f);
    *az = (int16_t)((float)raw_az * (9.80665f / 16384.0f) * 1000.0f);

    int16_t raw_gx = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t raw_gy = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t raw_gz = (int16_t)((buf[11] << 8) | buf[10]);
    /* rad/s × 1000: +/-2000 dps range → scale = (2000/32768) × (π/180) × 1000 */
    *gx = (int16_t)((float)raw_gx * (3.14159265f / (180.0f * 16.384f)) * 1000.0f);
    *gy = (int16_t)((float)raw_gy * (3.14159265f / (180.0f * 16.384f)) * 1000.0f);
    *gz = (int16_t)((float)raw_gz * (3.14159265f / (180.0f * 16.384f)) * 1000.0f);
}

static void read_mlx90393(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t sb_cmd = MLX_CMD_SB;
    i2c_write(i2c_dev2, &sb_cmd, 1, MLX90393_ADDR);
    k_msleep(10); /* Conversion time */

    uint8_t rm_cmd = MLX_CMD_RM;
    uint8_t buf[7];
    if (i2c_write_read(i2c_dev2, MLX90393_ADDR, &rm_cmd, 1, buf, 7) == 0) {
        /* µT × 100 (simplified scale) */
        *mx = (int16_t)(((buf[1] << 8) | buf[2]) / 10);
        *my = (int16_t)(((buf[3] << 8) | buf[4]) / 10);
        *mz = (int16_t)(((buf[5] << 8) | buf[6]) / 10);
    }
}

/* ── GPS UART ISR ─────────────────────────────────────────────────────────── */
static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    if (!uart_irq_update(dev)) {
        return;
    }
    while (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {
            if (c == '\n' || c == '\r') {
                if (gps_rx_idx > 0) {
                    gps_rx_buf[gps_rx_idx] = '\0';
                    gps_line_ready = true;
                }
            } else if (gps_rx_idx < (int)sizeof(gps_rx_buf) - 1) {
                gps_rx_buf[gps_rx_idx++] = c;
            }
        }
    }
}

static void parse_nmea(const char *line)
{
    if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
        /* Minimal stub — full NMEA parser TBD */
        gps_lat = 37.5665;
        gps_lon = 126.9780;
    }
}

/* ── CAN TX Helpers ───────────────────────────────────────────────────────── */
static void can_send_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame frame = {0};
    frame.id  = id;
    frame.dlc = dlc;
    memcpy(frame.data, data, dlc);
    can_send(can_dev, &frame, K_MSEC(5), NULL, NULL);
}

static void publish_imu(void)
{
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    read_bmi270(&ax, &ay, &az, &gx, &gy, &gz);

    uint8_t accel_buf[6];
    accel_buf[0] = (uint8_t)(ax >> 8); accel_buf[1] = (uint8_t)(ax & 0xFF);
    accel_buf[2] = (uint8_t)(ay >> 8); accel_buf[3] = (uint8_t)(ay & 0xFF);
    accel_buf[4] = (uint8_t)(az >> 8); accel_buf[5] = (uint8_t)(az & 0xFF);
    can_send_frame(CAN_ID_ACCEL, accel_buf, 6);

    uint8_t gyro_buf[6];
    gyro_buf[0] = (uint8_t)(gx >> 8); gyro_buf[1] = (uint8_t)(gx & 0xFF);
    gyro_buf[2] = (uint8_t)(gy >> 8); gyro_buf[3] = (uint8_t)(gy & 0xFF);
    gyro_buf[4] = (uint8_t)(gz >> 8); gyro_buf[5] = (uint8_t)(gz & 0xFF);
    can_send_frame(CAN_ID_GYRO, gyro_buf, 6);
}

static void publish_mag(void)
{
    int16_t mx = 0, my = 0, mz = 0;
    read_mlx90393(&mx, &my, &mz);

    uint8_t buf[6];
    buf[0] = (uint8_t)(mx >> 8); buf[1] = (uint8_t)(mx & 0xFF);
    buf[2] = (uint8_t)(my >> 8); buf[3] = (uint8_t)(my & 0xFF);
    buf[4] = (uint8_t)(mz >> 8); buf[5] = (uint8_t)(mz & 0xFF);
    can_send_frame(CAN_ID_MAG, buf, 6);
}

static void publish_encoder(void)
{
    float az_angle = read_as5600_angle(i2c_dev1);
    float el_angle = read_as5600_angle(i2c_dev2);

    int16_t az_raw = (az_angle >= 0.0f) ? (int16_t)(az_angle * 10.0f) : -1;
    int16_t el_raw = (el_angle >= 0.0f) ? (int16_t)(el_angle * 10.0f) : -1;
    uint8_t flags  = 0;
    if (az_angle >= 0.0f) { flags |= 0x01; }
    if (el_angle >= 0.0f) { flags |= 0x02; }

    uint8_t buf[5];
    buf[0] = (uint8_t)(az_raw >> 8); buf[1] = (uint8_t)(az_raw & 0xFF);
    buf[2] = (uint8_t)(el_raw >> 8); buf[3] = (uint8_t)(el_raw & 0xFF);
    buf[4] = flags;
    can_send_frame(CAN_ID_ENCODER, buf, 5);
}

static void publish_gps(void)
{
    int32_t lat_raw = (int32_t)(gps_lat * 1e7);
    int32_t lon_raw = (int32_t)(gps_lon * 1e7);
    uint8_t buf[8];
    memcpy(buf,     &lat_raw, 4);
    memcpy(buf + 4, &lon_raw, 4);
    can_send_frame(CAN_ID_GPS, buf, 8);
}

static void publish_heartbeat(void)
{
    uint32_t uptime = (uint32_t)k_uptime_get();
    uint8_t buf[5];
    memcpy(buf, &uptime, 4);
    buf[4] = 1; /* status: OK */
    can_send_frame(CAN_ID_HEARTBEAT, buf, 5);
}

/* ── Motor CMD Thread (CAN RX 0x300) ─────────────────────────────────────── */
static void motor_cmd_thread(void *p1, void *p2, void *p3)
{
    struct can_frame frame;

    while (1) {
        if (k_msgq_get(&motor_msgq, &frame, K_FOREVER) != 0) {
            continue;
        }
        if (frame.dlc < 5) {
            continue;
        }

        int16_t az_freq_raw, el_freq_raw;
        memcpy(&az_freq_raw, &frame.data[0], 2);
        memcpy(&el_freq_raw, &frame.data[2], 2);
        uint8_t flags = frame.data[4];

        if (flags & 0x04) { /* bit2 = estop */
            motor_estop = true;
            gpio_pin_set_dt(&az_en, 0);
            gpio_pin_set_dt(&el_en, 0);
            current_az_freq = 0.0f;
            current_el_freq = 0.0f;
            continue;
        }

        motor_estop = false;
        gpio_pin_set_dt(&az_en, 1);
        gpio_pin_set_dt(&el_en, 1);
        gpio_pin_set_dt(&az_dir, (flags & 0x01) ? 1 : 0); /* bit0 = az_dir */
        gpio_pin_set_dt(&el_dir, (flags & 0x02) ? 1 : 0); /* bit1 = el_dir */
        current_az_freq = (float)az_freq_raw / 10.0f;
        current_el_freq = (float)el_freq_raw / 10.0f;
    }
}
K_THREAD_DEFINE(motor_tid, 1024, motor_cmd_thread, NULL, NULL, NULL, 5, 0, 0);

/* ── Hardware Init ────────────────────────────────────────────────────────── */
static void init_hardware(void)
{
    /* GPIO: stepper motors (disabled by default) */
    gpio_pin_configure_dt(&az_step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&az_dir,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&az_en,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_set_dt(&az_en, 0);

    gpio_pin_configure_dt(&el_step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&el_dir,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&el_en,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_set_dt(&el_en, 0);

    /* I2C sensors */
    if (!device_is_ready(i2c_dev1)) { printk("I2C1 fail\n"); }
    if (!device_is_ready(i2c_dev2)) { printk("I2C2 fail\n"); }

    if (init_bmi270() == 0 && init_mlx90393() == 0) {
        printk("BMI270 + MLX90393 OK\n");
    } else {
        printk("Sensor init failed!\n");
    }

    /* GPS UART */
    if (uart_dev != NULL && device_is_ready(uart_dev)) {
        uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
        uart_irq_rx_enable(uart_dev);
        printk("GPS UART OK\n");
    } else {
        printk("GPS UART not available\n");
    }

    /* CAN */
    if (!device_is_ready(can_dev)) {
        printk("CAN device not ready!\n");
    } else {
        can_start(can_dev);

        static const struct can_filter motor_filter = {
            .id    = CAN_ID_MOTOR_CMD,
            .mask  = CAN_STD_ID_MASK,
            .flags = 0,
        };
        can_add_rx_filter_msgq(can_dev, &motor_msgq, &motor_filter);
        printk("CAN OK (500kbps, FDCAN1)\n");
    }

    /* Stepper pulse timer: 10kHz */
    k_timer_start(&stepper_timer, K_USEC(100), K_USEC(100));
}

/* ── Main ─────────────────────────────────────────────────────────────────── */
int main(void)
{
    printk("Antenna Tracker Firmware — CAN bus mode (STM32H7 FDCAN1)\n");
    init_hardware();

    uint64_t last_hb_ms  = 0;
    uint64_t last_gps_ms = 0;

    while (1) {
        /* 100Hz: IMU, mag, encoder */
        publish_imu();
        publish_mag();
        publish_encoder();

        uint64_t now_ms = k_uptime_get();

        /* GPS at 1Hz */
        if (now_ms - last_gps_ms >= 1000) {
            if (gps_line_ready) {
                parse_nmea(gps_rx_buf);
                gps_rx_idx    = 0;
                gps_line_ready = false;
            }
            publish_gps();
            last_gps_ms = now_ms;
        }

        /* Heartbeat at 1Hz */
        if (now_ms - last_hb_ms >= 1000) {
            publish_heartbeat();
            last_hb_ms = now_ms;
        }

        k_msleep(10); /* 100Hz loop */
    }

    return 0;
}
