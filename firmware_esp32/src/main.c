#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <string.h>

/* ── CAN Protocol IDs ─────────────────────────────────────────────────────── */
/* TX: ESP32 → RPi4B (via shared CAN bus) */
#define CAN_ID_TARGET_GPS    0x100  /* lat×1e7, lon×1e7 (int32 LE), 8 bytes */
#define CAN_ID_TARGET_STATUS 0x101  /* alt(int16 LE) + rssi(int16 LE) + lq(uint8), 5 bytes */

/* ── LoRa Configuration ───────────────────────────────────────────────────── */
/*
 * LoRa 수신 패킷 형식 (드론/기구에서 송신):
 *   [0-3]  lat  int32 LE  위도  × 1e7
 *   [4-7]  lon  int32 LE  경도  × 1e7
 *   [8-9]  alt  int16 LE  고도  (m)
 *   [10]   stat uint8     상태 플래그 (bit0=GPS fix, bit1=arm)
 *   총 11 bytes
 */
#define LORA_PACKET_SIZE     11
#define LORA_FREQ_HZ         923000000  /* 한국 ISM 923MHz */
#define LORA_TX_POWER        17         /* dBm (PA_BOOST) */

/* ── Devices ──────────────────────────────────────────────────────────────── */
static const struct device *can_dev  = DEVICE_DT_GET(DT_NODELABEL(twai));
static const struct device *lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora0));

/* ── CAN TX Helper ────────────────────────────────────────────────────────── */
static void can_send_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame frame = {0};
    frame.id  = id;
    frame.dlc = dlc;
    memcpy(frame.data, data, dlc);
    can_send(can_dev, &frame, K_MSEC(5), NULL, NULL);
}

/* ── LoRa → CAN Bridge ────────────────────────────────────────────────────── */
static void process_lora_packet(const uint8_t *buf, int len, int16_t rssi)
{
    if (len < LORA_PACKET_SIZE) {
        printk("LoRa: short packet (%d bytes), discarded\n", len);
        return;
    }

    int32_t lat, lon;
    int16_t alt;
    uint8_t stat;

    memcpy(&lat,  &buf[0], 4);
    memcpy(&lon,  &buf[4], 4);
    memcpy(&alt,  &buf[8], 2);
    stat = buf[10];

    /* 0x100: Target GPS */
    uint8_t gps_buf[8];
    memcpy(gps_buf,     &lat, 4);
    memcpy(gps_buf + 4, &lon, 4);
    can_send_frame(CAN_ID_TARGET_GPS, gps_buf, 8);

    /* 0x101: Target Status (alt + rssi + link_quality) */
    uint8_t status_buf[5];
    memcpy(status_buf,     &alt,  2);
    memcpy(status_buf + 2, &rssi, 2);
    status_buf[4] = stat;
    can_send_frame(CAN_ID_TARGET_STATUS, status_buf, 5);

    printk("LoRa RX → CAN: lat=%d lon=%d alt=%d rssi=%d stat=%u\n",
           lat, lon, (int)alt, (int)rssi, stat);
}

/* ── Main ─────────────────────────────────────────────────────────────────── */
int main(void)
{
    printk("ESP32 LoRa-CAN Bridge — Zephyr RTOS (TTGO LoRa32 V2.1)\n");
    printk("CAN: TWAI 500kbps (TX=GPIO32, RX=GPIO33)\n");
    printk("LoRa: SX1276 %.0fMHz SF7 BW125 CR4/5\n",
           LORA_FREQ_HZ / 1e6);

    /* ── CAN init ────────────────────────────────────────────────────────── */
    if (!device_is_ready(can_dev)) {
        printk("TWAI CAN device not ready!\n");
        return -1;
    }
    can_start(can_dev);
    printk("TWAI CAN OK\n");

    /* ── LoRa init ───────────────────────────────────────────────────────── */
    if (!device_is_ready(lora_dev)) {
        printk("LoRa device not ready!\n");
        return -1;
    }

    struct lora_modem_config lora_cfg = {
        .frequency    = LORA_FREQ_HZ,
        .bandwidth    = BW_125_KHZ,
        .datarate     = SF_7,
        .preamble_len = 8,
        .coding_rate  = CR_4_5,
        .tx_power     = LORA_TX_POWER,
        .tx           = false,   /* RX 모드 */
    };
    if (lora_config(lora_dev, &lora_cfg) < 0) {
        printk("LoRa config failed!\n");
        return -1;
    }
    printk("LoRa OK — listening on %.0f MHz\n", LORA_FREQ_HZ / 1e6);

    /* ── RX Loop ─────────────────────────────────────────────────────────── */
    uint8_t buf[64];
    int16_t rssi;
    int8_t  snr;

    while (1) {
        /* Blocking receive, 1s timeout → 계속 polling */
        int len = lora_recv(lora_dev, buf, sizeof(buf), K_MSEC(1000),
                            &rssi, &snr);
        if (len > 0) {
            process_lora_packet(buf, len, rssi);
        }
        /* timeout(len==0) or error(len<0): 그냥 재시도 */
    }

    return 0;
}
