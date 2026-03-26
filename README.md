# Antenna Tracker ROS 2 Humble

[![CI/CD](https://github.com/MINI-UNIQ-STAR/Antenna_Tracker_ROS2_humble1/actions/workflows/ci_cd.yml/badge.svg)](https://github.com/MINI-UNIQ-STAR/Antenna_Tracker_ROS2_humble1/actions)
[![cppcheck](https://github.com/MINI-UNIQ-STAR/Antenna_Tracker_ROS2_humble1/actions/workflows/cppcheck.yml/badge.svg)](https://github.com/MINI-UNIQ-STAR/Antenna_Tracker_ROS2_humble1/actions)

**야기 안테나(13dBi, 빔폭 ±2°) 자동 추적 시스템**
STM32H7 Nucleo(Zephyr RTOS) + RPi4B(Ubuntu 22.04 PREEMPT_RT) + Acados NMPC
**단일 CAN 2.0B 버스(500kbps)** 로 ESP32 · STM32H7 · RPi4B 3노드 통신

---

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                    CAN 2.0B Bus  500kbps                        │
│                                                                  │
│  [ESP32 LoRa]          [STM32H7 Nucleo]        [RPi4B]          │
│  TWAI (내장CAN)         FDCAN1                  MCP2515 SPI-HAT  │
│  0x100 Target GPS   →  0x200 Accel        →   can_bridge_node  │
│  0x101 Target Status   0x201 Gyro             ├─ /imu/raw       │
│  0x102 UTC/Status       0x202 Mag              ├─ /magnetic_field│
│  0x103 Accel/GyroX      0x203 GPS fix          ├─ /gps/fix       │
│  0x104 GyroYZ/MagXY     0x204 Encoder          ├─ /encoder_fb    │
│  0x105 MagZ/Roll/Pitch  0x205 Heartbeat        ├─ /target_gps   │
│  0x106 Temperature                             └─ /balloon/telem │
│  0x107 Pressure/CO2    ←   0x300 Motor Cmd                      │
│  0x108 Air Quality                                               │
│  0x109 Radiation/Bat                                             │
│  0x10A Seq/Uptime                                                │
└─────────────────────────────────────────────────────────────────┘

RPi4B ROS 2 Nodes:
  sensor_fusion_node  → Comp Filter (α=0.98) + Kalman → /antenna/imu_fusion
  navigation_node     → az/el 각도 계산
  controller_node     → Acados NMPC 100Hz → /antenna/motor_cmd
  state_machine_node  → AUTO / MANUAL / STANDBY / EMERGENCY
  can_bridge_node     → CAN ↔ ROS 2 변환

[Browser] ── rosbridge :9090 ──▶ GCS 웹 대시보드 (:8080)
[Gazebo Fortress] ── ros_gz_bridge ──▶ SITL 시뮬레이션
```

---

## 하드웨어 스펙

| 구성요소 | 모델 | 인터페이스 | 주소/핀 |
|---------|------|-----------|---------|
| MCU | STM32H7A3ZI-Q Nucleo | FDCAN1 (CAN 버스) | PD0(RX), PD1(TX) |
| IMU | BMI270 | I2C1 | 0x68 (SDO=GND) |
| 지자기계 | MLX90393 | I2C2 | 0x0C (A0=A1=0) |
| GPS | XA1110 | USART1 | PA9(TX), PA10(RX), 9600 baud |
| 인코더 (방위각) | AS5600 | I2C1 | 0x36 |
| 인코더 (고각) | AS5600 | I2C2 | 0x36 |
| 모터 드라이버 | TB6600 x2 | GPIO | AZ: PD14/PD15/PE9, EL: PE11/PF14/PE13 |
| CAN (RPi4B) | MCP2515 SPI-HAT | SPI0 | CS→GPIO8, INT→GPIO25 |
| CAN (ESP32) | TWAI (내장) | — | 500kbps CAN 2.0B |
| LoRa | ESP32 내장 | — | Target GPS 수신 |
| 안테나 | 야기 13dBi | — | 빔폭 4°, 추적 정밀도 ±1° |

**I2C 버스 배선:**
```
I2C1 (PB8/PB9): AS5600@0x36 (방위각 엔코더), BMI270@0x68 (IMU)
I2C2 (PB10/PB11): AS5600@0x36 (고각 엔코더), MLX90393@0x0C (지자기계)
```

---

## 소프트웨어 스택

| 구성 | 버전 |
|------|------|
| ROS 2 | Humble |
| 펌웨어 RTOS | Zephyr RTOS (Zephyr SDK 0.16.8) |
| 제어 알고리즘 | Acados NMPC (비선형 모델 예측 제어) |
| 센서 퓨전 | Complementary Filter (α=0.98) + Linear Kalman |
| 시뮬레이터 | Gazebo Fortress (`ros_gz_bridge`) |
| CI/CD | GitHub Actions + GHCR Docker |
| OS (RPi4B) | Ubuntu 22.04 + PREEMPT_RT 커널 |

---

## 패키지 구조

```
src/
├── antenna_tracker_msgs/       # 커스텀 메시지/서비스/액션
│   └── msg/BalloonTelemetry.msg  # Space Balloon 텔레메트리 통합 메시지 (59필드)
├── antenna_tracker_hardware/   # CAN 브릿지 노드 (SocketCAN ↔ ROS 2)
├── antenna_tracker_controller/ # NMPC, 센서퓨전, 네비게이션, 상태머신
├── antenna_tracker_simulation/ # Gazebo Fortress URDF + sim_motor_bridge
├── antenna_tracker_bringup/    # 런치 파일 + 파라미터
└── antenna_tracker_web/        # GCS 웹 대시보드 (rosbridge + HTTP)

firmware/                       # STM32H7 Nucleo — Zephyr RTOS CAN 펌웨어
  ├── src/main.c                # CAN TX/RX + BMI270/MLX90393/AS5600/GPS + 스테퍼
  ├── app.overlay               # 디바이스 트리 오버레이 (I2C1/2, FDCAN1, USART1)
  └── prj.conf                  # Zephyr Kconfig (CONFIG_CAN=y)

firmware_esp32/                 # TTGO LoRa 32 V2.1 — Zephyr RTOS LoRa-CAN 펌웨어
  ├── src/main.c                # LoRa RX (SX1276, 915MHz SF11) → telemetry_frame_t 파싱
  │                             # → CAN TX (TWAI, 0x100~0x10A, 11개 프레임)
  ├── app.overlay               # 디바이스 트리 오버레이 (SPI2=SX1276 LoRa, TWAI GPIO32/33=CAN)
  └── prj.conf                  # Zephyr Kconfig (CONFIG_LORA=y, CONFIG_CAN=y)

scripts/                        # 설치/배포 자동화 스크립트
```

---

## 설치

### 사전 요구사항

- RPi4B: Ubuntu 22.04 (PREEMPT_RT 커널 권장)
- ROS 2 Humble
- Zephyr SDK 0.16.8 (펌웨어 빌드 시)
- Docker (개발/SITL 환경)

### RPi4B 환경 설정 (최초 1회)

```bash
cd scripts
bash setup_ros2_humble.sh      # ROS 2 Humble + Acados NMPC 라이브러리
bash install_preempt_rt.sh     # PREEMPT_RT 커널 빌드 (~2시간) → reboot 필요
bash setup_can.sh              # MCP2515 dtoverlay 설정 + can0 활성화
```

> ⚠️ MCP2515 오실레이터가 8MHz/12MHz인 경우 `setup_can.sh`에서 `oscillator=16000000`을 실제 값으로 수정하세요.

### MCP2515 배선 (RPi4B)

```
MCP2515 VCC  → Pin 17 (3.3V)
MCP2515 GND  → Pin 20 (GND)
MCP2515 SCK  → Pin 23 (GPIO11, SPI0_CLK)
MCP2515 MOSI → Pin 19 (GPIO10, SPI0_MOSI)
MCP2515 MISO → Pin 21 (GPIO9,  SPI0_MISO)
MCP2515 CS   → Pin 24 (GPIO8,  SPI0_CE0)
MCP2515 INT  → Pin 22 (GPIO25)
```

### ROS 2 워크스페이스 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 펌웨어 빌드 및 플래시

#### Zephyr SDK 설치 (최초 1회, Linux/WSL2 공통)

```bash
# Python 가상환경 + west
python3 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate
pip install west

# Zephyr 소스 및 SDK
west init ~/zephyrproject
cd ~/zephyrproject
west update
west zephyr-export
pip install -r zephyr/scripts/requirements.txt

# Zephyr SDK 0.16.8 다운로드 및 설치
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz
tar xf zephyr-sdk-0.16.8_linux-x86_64.tar.xz -C ~/zephyrproject/
cd ~/zephyrproject/zephyr-sdk-0.16.8
./setup.sh
```

---

#### STM32H7 Nucleo 플래시

##### Linux (권장)

```bash
# ST-Link udev 규칙 등록 (최초 1회)
sudo cp ~/zephyrproject/zephyr-sdk-0.16.8/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload

# 빌드 및 플래시
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh

cd firmware
west build -b nucleo_h7a3zi_q
west flash   # Nucleo 내장 ST-Link 자동 감지
```

##### Windows (STM32CubeProgrammer)

1. [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) 설치
2. Nucleo USB 연결 → ST-LINK 드라이버 자동 설치
3. STM32CubeProgrammer 실행:
   - Interface: **ST-LINK** → Connect
   - `firmware/build/zephyr/zephyr.bin` 파일 선택
   - Start address: `0x08000000`
   - **Download** 클릭

##### Windows (west + usbipd-win)

```powershell
# 1. usbipd-win 설치 (PowerShell 관리자)
winget install usbipd

# 2. Nucleo 연결 후 장치 확인
usbipd list
# → STMicroelectronics STLink Virtual COM Port 항목의 BUSID 확인 (예: 2-3)

# 3. WSL2로 연결
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3

# 4. WSL2 터미널에서 west flash
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh
cd firmware
west flash
```

---

#### TTGO LoRa 32 V2.1 (ESP32) 플래시

##### Linux (권장)

```bash
# CP210x USB-Serial 드라이버 확인
ls /dev/ttyUSB*   # → /dev/ttyUSB0 확인
sudo usermod -aG dialout $USER   # 권한 추가 (재로그인 필요)

# 빌드 및 플래시
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh

cd firmware_esp32
west build -b esp32_devkitc_wroom/esp32/procpu
west flash --esp-device /dev/ttyUSB0

# 시리얼 모니터
west espressif monitor --esp-device /dev/ttyUSB0
```

##### Windows (esptool.py — 가장 간단)

```powershell
# 1. Python + esptool 설치
pip install esptool

# 2. TTGO 연결 → 장치 관리자에서 COM 포트 확인 (예: COM5)

# 3. 빌드된 바이너리 위치
#    firmware_esp32/build/zephyr/zephyr.bin

# 4. 플래시
esptool.py --chip esp32 --port COM5 --baud 921600 `
  write_flash `
  0x1000  firmware_esp32/build/zephyr/bootloader/esp-idf/components/bootloader/subproject/main/bootloader.bin `
  0x8000  firmware_esp32/build/zephyr/partitions_singleapp.bin `
  0x10000 firmware_esp32/build/zephyr/zephyr.bin
```

##### Windows (west + usbipd-win)

```powershell
# 1. TTGO 연결 후 BUSID 확인
usbipd list   # → Silicon Labs CP210x ... BUSID 확인 (예: 3-1)

# 2. WSL2로 연결
usbipd bind --busid 3-1
usbipd attach --wsl --busid 3-1

# 3. WSL2 터미널에서 west flash
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh
cd firmware_esp32
west flash --esp-device /dev/ttyUSB0
```

> ⚠ TTGO TWAI는 외부 CAN 트랜시버(SN65HVD230 등)가 필요합니다.
> GPIO32(TX), GPIO33(RX)에 트랜시버 연결 후 CAN 버스에 합류하세요.
>
---

## 펌웨어 이식 가이드 (STM32H7 → 다른 보드)

`firmware/src/main.c`는 Zephyr DT API를 사용하므로 **로직 수정 없이** 아래 파일만 수정하면 됩니다.

### 공통 수정 항목

| 항목 | 기존 (STM32H7 Nucleo) | 변경 방법 |
|------|----------------------|----------|
| 빌드 커맨드 | `west build -b nucleo_h7a3zi_q` | `-b <새 보드명>` |
| `app.overlay` CAN 노드 | `&fdcan1` | 보드별 CAN 노드명으로 |
| `app.overlay` GPIO/I2C/UART 핀 | PD0, PB8 등 STM32 핀 | 새 보드 핀으로 재매핑 |

---

### ESP32-C6 로 이식

> Zephyr 3.5+ 지원 / RISC-V 기반 / TWAI(CAN 2.0B) 내장

**1. 빌드**
```bash
west build -b esp32c6_devkitc/esp32c6
west flash --esp-device /dev/ttyUSB0
```

**2. `app.overlay` 핵심 변경**

```dts
/* CAN: FDCAN1 → TWAI (ESP32-C6 내장) */
chosen {
    zephyr,canbus = &twai;
};

&twai {
    status = "okay";
    pinctrl-0 = <&twai_tx_gpioXX &twai_rx_gpioXX>;  /* 실제 연결 핀으로 */
    pinctrl-names = "default";
    bus-speed = <500000>;
};

/* I2C, UART, GPIO 핀 번호를 ESP32-C6 GPIO 번호로 재매핑 */
&i2c0 {
    pinctrl-0 = <&i2c0_sda_gpioXX &i2c0_scl_gpioXX>;
    ...
};
```

**3. `prj.conf`** — 변경 없음 (`CONFIG_CAN=y` 그대로 사용)

---

### Arduino UNO R4 Minima 로 이식

> Renesas RA4M1 (ARM Cortex-M4) / CAN0 내장 / Zephyr 3.5+ 지원

**1. 빌드**
```bash
west build -b arduino_uno_r4_minima
west flash   # OpenOCD or J-Link
```

**2. `app.overlay` 핵심 변경**

```dts
/* CAN: FDCAN1 → RA4M1 CAN0 */
chosen {
    zephyr,canbus = &can0;
};

&can0 {
    status = "okay";
    pinctrl-0 = <&can0_tx_p103 &can0_rx_p102>;  /* RA4M1 기본 CAN 핀 */
    pinctrl-names = "default";
    bus-speed = <500000>;
};

/* I2C: Arduino 표준 SDA(A4)/SCL(A5) */
&i2c1 {
    pinctrl-0 = <&i2c1_sda_p407 &i2c1_scl_p408>;
    ...
};

/* UART: Arduino D0(RX)/D1(TX) */
&uart2 {
    pinctrl-0 = <&uart2_rx_p301 &uart2_tx_p302>;
    current-speed = <9600>;
    ...
};
```

**3. `prj.conf`** — 변경 없음

> ⚠️ 정확한 핀 번호는 [RA4M1 데이터시트](https://www.renesas.com/us/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra4m1-32-bit-microcontrollers-48mhz-arm-cortex-m4) 및 보드 회로도를 반드시 대조하세요.

---

### 이식 체크리스트

- [ ] `west build -b <보드명>` 성공
- [ ] CAN 노드명 (`&fdcan1` / `&twai` / `&can0`) 확인
- [ ] I2C1/I2C2 핀 재매핑 (BMI270@0x68, MLX90393@0x0C, AS5600×2@0x36)
- [ ] UART 핀 재매핑 (GPS XA1110, 9600 baud)
- [ ] 스테퍼 GPIO 6핀 재매핑 (STEP/DIR/EN × 2축)
- [ ] `candump` 또는 vcan SITL로 CAN 통신 확인

---

## SITL (Software In The Loop) — vcan + Docker

하드웨어 없이 PC에서 전체 소프트웨어 스택을 검증하는 방법입니다.

### 1. vcan 인터페이스 설정

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 2. Docker 이미지 준비

```bash
# 방법 A: GHCR에서 pre-built 이미지 pull (권장)
docker pull ghcr.io/hyeonsuparkembedded/antenna_tracker_env:latest
docker tag ghcr.io/hyeonsuparkembedded/antenna_tracker_env:latest antenna_tracker_env

# 방법 B: 로컬에서 직접 빌드
docker build -t antenna_tracker_env docker/
```

### 3. Docker 컨테이너 실행

```bash
cd docker
xhost +local:docker   # Gazebo GUI 필요 시
docker compose up -d
docker exec -it antenna_tracker_dev bash
```

### 3. ROS 2 빌드 및 실행

```bash
# 컨테이너 내부
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# CAN 브릿지 (vcan0 사용)
ros2 run antenna_tracker_hardware can_bridge_node \
  --ros-args -p can_interface:=vcan0

# 별도 터미널: 전체 스택 실행
ros2 launch antenna_tracker_bringup hardware.launch.py can_interface:=vcan0
```

### 4. 가상 센서 데이터 주입 (cansend)

```bash
# 엔코더: az=288.0°, el=90.0°
cansend vcan0 204#0B40038403

# IMU 가속도: ax=1.0 m/s² (int16 × 1000, Big-Endian)
cansend vcan0 200#03E8000003E8000000000000

# IMU 자이로: gx=0.1 rad/s (int16 × 1000)
cansend vcan0 201#006400640064

# 지자기: mx=30µT (int16 × 100)
cansend vcan0 202#0BB80BB80BB8

# GPS: lat=37.5665°, lon=126.978°
cansend vcan0 203#6396C1011A7B9007

# Target GPS (ESP32 시뮬): lat=37.57°, lon=126.98°
cansend vcan0 100#A0FF370107807900
```

#### Balloon Telemetry 시뮬 (0x100~0x10A, 11개 프레임)

ESP32가 LoRa로 수신한 Space Balloon 텔레메트리를 시뮬하려면 11개 프레임을 모두 전송해야
`/balloon/telemetry` 토픽이 발행됩니다.

```bash
# 0x100: lat×1e7=0, lon×1e7=0
cansend vcan0 100#0000000000000000
# 0x101: kf_alt=0, rssi=0, fix=1, sats=8, h=12, m=0
cansend vcan0 101#0000000001080C00
# 0x102: sec=0, day=1, mon=3, sats_view=8, year=2026(LE), flags=0
cansend vcan0 102#00010308EA070000
# 0x103: accel_xyz=0(int16×100), gyro_x=0(int16×1000)
cansend vcan0 103#0000000000000000
# 0x104: gyro_yz=0, mag_xy=0(int16×10)
cansend vcan0 104#0000000000000000
# 0x105: mag_z=0, kf_roll=0, kf_pitch=0, press_alt=0
cansend vcan0 105#0000000000000000
# 0x106: board/ext/sht31 temp=0, sht31_rh=0
cansend vcan0 106#0000000000000000
# 0x107: ms5611_press=0(u32), ms5611_temp=0, co2=400ppm(LE)
cansend vcan0 107#0000000000009001
# 0x108: pm1/pm25/pm10=0, ozone=0
cansend vcan0 108#0000000000000000
# 0x109: gdk101=0, bat_mv=3700(LE), bat_temp=0, heater=0
cansend vcan0 109#0000740E00000000
# 0x10A: seq=1(LE), uptime=1000ms(LE)
cansend vcan0 10A#0100E80300000000
```

### 5. 토픽 확인

```bash
ros2 topic echo /antenna/encoder_feedback
ros2 topic echo /imu/raw
ros2 topic echo /magnetic_field
ros2 topic echo /antenna/imu_fusion
ros2 topic echo /balloon/telemetry
```

### 6. Gazebo Fortress 시뮬레이션

```bash
# 컨테이너 내부 (Xvfb 헤드리스 or DISPLAY 필요)
ros2 launch antenna_tracker_bringup sim.launch.py
```

---

## HITL (Hardware In The Loop) — 실제 하드웨어

실제 STM32H7 보드와 CAN 버스를 연결하여 ROS 2와 통신하는 방법입니다.

> RPi4B에서 Docker로 실행하는 경우 개발 환경 이미지를 pull해 사용할 수 있습니다:
> ```bash
> docker pull ghcr.io/hyeonsuparkembedded/antenna_tracker_env:latest
> docker tag ghcr.io/hyeonsuparkembedded/antenna_tracker_env:latest antenna_tracker_env
> ```

### 1. 펌웨어 플래시

```bash
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh

# STM32H7 Nucleo
cd firmware
west build -b nucleo_h7a3zi_q
west flash

# TTGO LoRa 32 V2.1
cd ../firmware_esp32
west build -b esp32_devkitc_wroom/esp32/procpu
west flash --esp-device /dev/ttyUSB0
```

### 2. CAN 버스 물리 연결

```
STM32H7 Nucleo ─── CAN 트랜시버 ─── CAN H/L 버스 ─── MCP2515 (RPi4B)
  PD1 (FDCAN1 TX)                                         SPI0
  PD0 (FDCAN1 RX)

ESP32 ──────────── CAN 트랜시버 ─── CAN H/L 버스 (동일 버스)
  TWAI TX
  TWAI RX

※ 버스 양단에 120Ω 종단 저항 필수
```

### 3. RPi4B에서 CAN 인터페이스 활성화

```bash
sudo ip link set can0 up type can bitrate 500000
ip link show can0    # 상태 확인

# 수신 모니터링
candump can0
```

### 4. ROS 2 실행

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# CAN 브릿지 실행 (실제 can0 사용)
ros2 launch antenna_tracker_bringup hardware.launch.py

# GCS 웹 대시보드
ros2 launch antenna_tracker_bringup web.launch.py
# → http://<RPi4B-IP>:8080
```

### 5. 실시간 실행 (PREEMPT_RT)

```bash
# controller_node를 core 2,3에 고정, RT 우선순위 80
sudo chrt -f 80 taskset -c 2,3 ros2 run antenna_tracker_controller controller_node

# 지터 측정 (목표: Max < 2ms)
sudo cyclictest -l100000 -m -Sp90 -i200 -h400 -q
```

---

## 주요 토픽

| 토픽 | 타입 | 주기 | 방향 |
|------|------|------|------|
| `/imu/raw` | sensor_msgs/Imu | 100Hz | STM32H7 → RPi4B |
| `/magnetic_field` | sensor_msgs/MagneticField | 100Hz | STM32H7 → RPi4B |
| `/gps/fix` | sensor_msgs/NavSatFix | 1Hz | STM32H7 → RPi4B |
| `/antenna/encoder_feedback` | EncoderFeedback | 100Hz | STM32H7 → RPi4B |
| `/antenna/target_gps` | TargetGPS | CAN 수신 시 | ESP32 → RPi4B |
| `/balloon/telemetry` | BalloonTelemetry | LoRa 수신 시 | ESP32 → RPi4B |
| `/antenna/imu_fusion` | ImuFusion | 100Hz | sensor_fusion_node |
| `/antenna/motor_cmd` | MotorCommand | 100Hz | RPi4B → STM32H7 |
| `/antenna/state` | AntennaState | 10Hz | state_machine_node |
| `/antenna/diagnostics` | TrackerDiagnostics | 1Hz | controller_node |

---

## 서비스 / 액션

```bash
# 모드 전환 (0=AUTO, 1=MANUAL, 2=STANDBY, 3=EMERGENCY)
ros2 service call /antenna/set_mode antenna_tracker_msgs/srv/SetMode "{mode: 0}"

# 수동 타겟 지정
ros2 service call /antenna/set_manual_target antenna_tracker_msgs/srv/SetManualTarget \
  "{azimuth_deg: 180.0, elevation_deg: 45.0}"

# 추적 액션
ros2 action send_goal /antenna/track_target antenna_tracker_msgs/action/TrackTarget \
  "{target_azimuth_deg: 180.0, target_elevation_deg: 45.0, tolerance_deg: 1.0, timeout_sec: 30.0}"
```

---

## CAN 프로토콜 요약

| CAN ID | 송신자 | 크기 | 내용 |
|--------|--------|------|------|
| 0x100 | ESP32 | 8B | Balloon lat×1e7(int32 LE), lon×1e7(int32 LE) |
| 0x101 | ESP32 | 8B | kf_alt(int16 LE), rssi(int16 LE), fix, sats_used, hour, min |
| 0x102 | ESP32 | 8B | sec, day, mon, sats_view, year(uint16 LE), status_flags(uint16 LE) |
| 0x103 | ESP32 | 8B | accel_x/y/z(int16 LE ×100 m/s²), gyro_x(int16 LE ×1000 rad/s) |
| 0x104 | ESP32 | 8B | gyro_y/z(int16 LE ×1000 rad/s), mag_x/y(int16 LE ×10 µT) |
| 0x105 | ESP32 | 8B | mag_z(int16 LE ×10 µT), kf_roll/pitch(int16 LE ×100 °), press_alt(int16 LE ×10 m) |
| 0x106 | ESP32 | 8B | board/ext/sht31 temp(int16 LE ×100 °C), sht31_rh(uint16 LE ×100 %) |
| 0x107 | ESP32 | 8B | ms5611_press(uint32 LE Pa), ms5611_temp(int16 LE ×100 °C), co2(uint16 LE ppm) |
| 0x108 | ESP32 | 8B | pm1/pm25/pm10(uint16 LE µg/m³), ozone(int16 LE ppb) |
| 0x109 | ESP32 | 8B | gdk101(uint16 LE ×100 µSv/h), bat_mv(uint16 LE), bat_temp(int16 LE ×100 °C), heater(uint8 × 2) |
| 0x10A | ESP32 | 8B | seq(uint16 LE), uptime_ms(uint32 LE), pad(uint16) |
| 0x200 | STM32H7 | 6B | Accel ax,ay,az (int16 BE ×1000 m/s²) |
| 0x201 | STM32H7 | 6B | Gyro gx,gy,gz (int16 BE ×1000 rad/s) |
| 0x202 | STM32H7 | 6B | Mag mx,my,mz (int16 BE ×100 µT) |
| 0x203 | STM32H7 | 8B | GPS lat×1e7, lon×1e7 (int32 LE) |
| 0x204 | STM32H7 | 5B | Encoder az×10, el×10 (int16 BE) + flags |
| 0x205 | STM32H7 | 5B | Heartbeat uptime_ms (uint32 LE) + status |
| 0x300 | RPi4B | 5B | Motor az_freq×10, el_freq×10 (int16 LE) + flags |

flags (0x300): bit0=az_dir, bit1=el_dir, bit2=estop

---

## 라이선스

MIT License — Hyeonsu Park (MINI-UNIQ-STAR)
