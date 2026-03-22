# Antenna Tracker ROS 2 Humble

[![ROS 2 Build](https://github.com/HyeonsuParkembedded/antenna_tracker_ros2_humble/actions/workflows/ci_cd.yml/badge.svg)](https://github.com/HyeonsuParkembedded/antenna_tracker_ros2_humble/actions)

**야기 안테나(13dBi, 빔폭 ±2°) 자동 추적 시스템** — STM32H7 Nucleo(micro-ROS) + RPi4B(Ubuntu 22.04 PREEMPT_RT) + Acados NMPC

---

## 시스템 구성

```
[ESP32 LoRa] ──CAN 500kbps──▶ [MCP2515 SPI-CAN] ──SPI──▶ [RPi4B]
(Target GPS)                   (RPi4B HAT)                  ROS 2 Humble
                                                              ├── sensor_fusion_node
[STM32H7 Nucleo] ─USB CDC─▶   micro_ros_agent               ├── navigation_node
  BMI270 (I2C1)                 /imu/raw                     ├── controller_node (NMPC 100Hz)
  MLX90393 (I2C1)               /magnetic_field              ├── state_machine_node
  XA1110 (UART)                 /gps/fix                     └── can_bridge_node
  AS5600 (I2C1/2)               /antenna/encoder_feedback    
  TB6600 Stepper                /antenna/motor_cmd ◀──       

[Browser] ─── rosbridge :9090 ──▶ GCS 웹 대시보드 (:8080)
[Gazebo Fortress] ─── ros_gz_bridge ──▶ 시뮬레이션
```

---

## 하드웨어 스펙

| 구성요소 | 모델 | 인터페이스 |
|---------|------|-----------|
| MCU | STM32H7A3ZI-Q Nucleo | micro-ROS USB CDC (`/dev/ttyACM0`) |
| IMU | BMI270 | I2C1 (0x68) |
| 지자기계 | MLX90393 | I2C1 (0x18) |
| GPS | XA1110 | UART (gps_uart alias) |
| 인코더 | AS5600 x2 | Dual-Bus I2C (Az: I2C1, El: I2C2) |
| 모터 드라이버 | TB6600 x2 | STEP/DIR/EN GPIO (10kHz k_timer) |
| CAN 컨트롤러 | **MCP2515** (SPI-CAN HAT) | SPI0, INT→GPIO25 |
| LoRa 모듈 | ESP32 LoRa | CAN 2.0B 500kbps |
| 안테나 | 야기 13dBi | 빔폭 4° (±2°), 추적 정밀도 ±1° |

---

## 소프트웨어 스택

| 구성 | 버전 |
|------|------|
| ROS 2 | Humble |
| micro-ROS | Humble (Zephyr RTOS) |
| 제어 알고리즘 | **Acados NMPC** (비선형 모델 예측 제어) |
| 센서 퓨전 | Complementary Filter (α=0.98) + Kalman (Q=0.001, R=2.0) |
| 시뮬레이터 | Gazebo Fortress (`ros_gz_bridge`) |
| CI/CD | GitHub Actions + GHCR Docker |
| OS (RPi4B) | Ubuntu 22.04 + PREEMPT_RT 커널 |

---

## 패키지 구조

```
src/
├── antenna_tracker_msgs/       # 커스텀 메시지/서비스/액션
├── antenna_tracker_hardware/   # CAN 브릿지 노드
├── antenna_tracker_controller/ # NMPC, 센서퓨전, 네비게이션, 상태머신
├── antenna_tracker_simulation/ # Gazebo Fortress URDF + sim_motor_bridge
├── antenna_tracker_bringup/    # 런치 파일 + 파라미터
└── antenna_tracker_web/        # GCS 웹 대시보드 (rosbridge + HTTP)

firmware/                       # STM32H7 Zephyr micro-ROS 펌웨어
scripts/                        # 설치/배포 자동화 스크립트
```

---

## 빠른 시작

### 1. RPi4B 환경 설정 (최초 1회)

```bash
cd scripts
bash setup_ros2_humble.sh      # ROS 2 Humble + Acados NMPC 라이브러리
bash install_preempt_rt.sh     # PREEMPT_RT 커널 빌드 (~2시간) → reboot 필요
bash setup_micro_ros.sh        # micro-ROS agent
bash setup_can.sh              # MCP2515 dtoverlay 설정 + can0 활성화
```

### 2. 워크스페이스 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 실행

```bash
# 하드웨어 (STM32 USB 연결 필요)
ros2 launch antenna_tracker_bringup hardware.launch.py

# 시뮬레이션 (Gazebo Fortress)
ros2 launch antenna_tracker_bringup sim.launch.py

# GCS 웹 대시보드 → http://<RPi4B-IP>:8080
ros2 launch antenna_tracker_bringup web.launch.py
```

### 4. 펌웨어 플래시 (STM32H7)

```bash
cd firmware
west build -b nucleo_h7a3zi_q
west flash
```

---

## 주요 토픽

| 토픽 | 타입 | 주기 |
|------|------|------|
| `/imu/raw` | sensor_msgs/Imu | 100Hz |
| `/magnetic_field` | sensor_msgs/MagneticField | 100Hz |
| `/gps/fix` | sensor_msgs/NavSatFix | 1Hz |
| `/antenna/encoder_feedback` | EncoderFeedback | 100Hz |
| `/antenna/motor_cmd` | MotorCommand | 100Hz |
| `/antenna/state` | AntennaState | 10Hz |
| `/antenna/target_gps` | TargetGPS | CAN 수신 시 |
| `/antenna/diagnostics` | TrackerDiagnostics | 1Hz |

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

## PREEMPT_RT 실시간 실행

```bash
# controller_node를 core 2,3에 고정, RT 우선순위 80
sudo chrt -f 80 taskset -c 2,3 ros2 run antenna_tracker_controller controller_node

# 지터 측정
sudo cyclictest -l100000 -m -Sp90 -i200 -h400 -q  # 목표: Max < 2ms
```

---

## MCP2515 배선 (RPi4B)

```
MCP2515 VCC  → Pin 17 (3.3V)
MCP2515 GND  → Pin 20 (GND)
MCP2515 SCK  → Pin 23 (GPIO11, SPI0_CLK)
MCP2515 MOSI → Pin 19 (GPIO10, SPI0_MOSI)
MCP2515 MISO → Pin 21 (GPIO9,  SPI0_MISO)
MCP2515 CS   → Pin 24 (GPIO8,  SPI0_CE0)
MCP2515 INT  → Pin 22 (GPIO25)
```

> ⚠️ MCP2515 모듈의 오실레이터가 8MHz 또는 12MHz인 경우 `setup_can.sh` 내
> `oscillator=16000000` 값을 실물에 맞게 수정하세요.

---

## 라이선스

MIT License — Hyeonsu Park (MINI-UNIQ-STAR)
