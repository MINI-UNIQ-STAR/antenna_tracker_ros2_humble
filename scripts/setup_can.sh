#!/bin/bash
# Antenna Tracker CAN Interface Setup Script
# MCP2515 SPI-CAN 컨트롤러 기반 (RPi4B에는 네이티브 CAN 포트 없음)
# 하드웨어: MCP2515 (SPI0, CS=CE0, INT=GPIO25, 오실레이터=16MHz 일반적)
set -e

echo "=== MCP2515 CAN Interface Setup ==="
echo "RPi4B does not have a native CAN port — using MCP2515 via SPI."
echo ""

# 1. 필수 패키지 설치
if ! command -v ip &> /dev/null || ! command -v candump &> /dev/null; then
    echo "--- Installing can-utils and iproute2 ---"
    sudo apt-get update && sudo apt-get install -y iproute2 can-utils
fi

# 2. SPI 활성화 및 MCP2515 Device Tree Overlay 설정
CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/config.txt"  # 구형 Ubuntu 경로 폴백
fi

echo "--- Checking /boot/firmware/config.txt for MCP2515 overlay ---"
if ! grep -q "mcp2515-can0" "$CONFIG_FILE"; then
    echo ""
    echo "  >> MCP2515 overlay not found. Adding to ${CONFIG_FILE}..."
    sudo tee -a "$CONFIG_FILE" <<'EOF'

# MCP2515 SPI-CAN Controller (Antenna Tracker)
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=spi-bcm2835
EOF
    echo "  ✔ Config written. A REBOOT is required."
    echo ""
    echo "  [MCP2515 Wiring]"
    echo "    MCP2515 VCC  → RPi4B Pin 17 (3.3V)"
    echo "    MCP2515 GND  → RPi4B Pin 20 (GND)"
    echo "    MCP2515 SCK  → RPi4B Pin 23 (GPIO11, SPI0_CLK)"
    echo "    MCP2515 MOSI → RPi4B Pin 19 (GPIO10, SPI0_MOSI)"
    echo "    MCP2515 MISO → RPi4B Pin 21 (GPIO9,  SPI0_MISO)"
    echo "    MCP2515 CS   → RPi4B Pin 24 (GPIO8,  SPI0_CE0)"
    echo "    MCP2515 INT  → RPi4B Pin 22 (GPIO25)"
    echo ""
    echo "  ⚠ If your MCP2515 has a different oscillator (8MHz or 12MHz),"
    echo "    change oscillator=16000000 in ${CONFIG_FILE} to match."
    echo ""
    read -r -p "Reboot now? [y/N] " ans
    if [[ "$ans" =~ ^[Yy]$ ]]; then sudo reboot; fi
    exit 0
else
    echo "  ✔ MCP2515 overlay already configured."
fi

# 3. can0 인터페이스 활성화
echo "--- Bringing up can0 at 500kbps ---"

if ! ip link show can0 &> /dev/null; then
    echo "ERROR: can0 interface not found!"
    echo "Make sure the RPi4B was rebooted after adding the overlay."
    exit 1
fi

if ip link show can0 | grep -q "UP"; then
    sudo ip link set can0 down
fi

sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

if ip link show can0 | grep -q "UP"; then
    echo ""
    echo "=== SUCCESS: can0 is UP at 500kbps ==="
    echo "Test reception: candump can0"
    echo "Test send:      cansend can0 123#DEADBEEF"
else
    echo "FAILED to bring up can0"
    exit 1
fi
