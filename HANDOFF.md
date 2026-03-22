# Antenna Tracker ROS2 - 에이전트 인수인계 문서

> **최종 업데이트**: 2026-03-21
> **프로젝트**: `antenna_tracker_ros` (ROS2 Humble + micro-ROS + Gazebo Fortress)
> **GitHub**: [HyeonsuParkembedded/antenna_tracker_ros2_humble](https://github.com/HyeonsuParkembedded/antenna_tracker_ros2_humble.git)
> **담당자**: Hyeonsu Park (MINI-UNIQ-STAR)

---

## 1. 프로젝트 배경

기존 `antenna_tracker_imu_encoder` 프로젝트(STM32H7 + Zephyr RTOS + Raspberry Pi 4B FastAPI)를 **ROS2 Humble + micro-ROS 풀 통합** 방식으로 재설계한 프로젝트.

### 핵심 설계 결정

| 항목 | 결정 |
|------|------|
| micro-ROS transport | UART (`usart3` → `/dev/ttyUSB0` 또는 `/dev/ttyACM0`) |
| RPi4B OS | Ubuntu 22.04 + PREEMPT_RT 커널 |
| 100Hz 제어 루프 | RPi4B `controller_node` (NMPC 기반) |
| 제어 알고리즘 | Acados NMPC (비선형 모델 예측 제어) |
| 인코더 방식 | Dual AS5600 (I2C1: Azimuth, I2C2: Elevation) |
| IMU 방식 | BMI270 (I2C1) + MLX90393 (I2C1) |
| GPS 방식 | XA1110 (UART: gps_uart alias) |
| 모터 구동 | TB6600 스텝퍼 (STEP/DIR/EN GPIO, 10kHz k_timer) |
| ESP32 LoRa 연결 | CAN 2.0B 500kbps (`MCP2515` SPI-CAN HAT, RPi4B SPI0) |
| 시뮬레이터 | Gazebo Fortress (ros_gz_bridge) |
| 웹 GCS | rosbridge WebSocket + Canvas 컴패스 |

### 전체 아키텍처

```
[STM32H7 Nucleo] ──USB CDC──────────────── [RPi4B Ubuntu 22.04 + PREEMPT_RT]
   Zephyr + micro-ROS Humble                  ROS2 Humble
  ├── BMI270(I2C1) → /imu/raw (100Hz)        ├── /micro_ros_agent (:ttyACM0)
  ├── MLX90393(I2C1) → /magnetic_field       ├── sensor_fusion_node  (100Hz)
  ├── XA1110(UART) → /gps/fix                ├── navigation_node     (Haversine)
  ├── AS5600(I2C1/2) → /antenna/encoder_feedback ├── controller_node (100Hz NMPC)
  ├── k_timer 10kHz → TB6600 Stepper         ├── state_machine_node  (AUTO/MANUAL/…)
  └── /antenna/motor_cmd ← motor_cmd_sub     
[ESP32 LoRa] ──CAN 500kbps──────────────→ can_bridge_node
                                              │
[Browser GCS] ──rosbridge :9090───────────→ /antenna/state, /antenna/motor_cmd, …
[Gazebo Fortress] ──ros_gz_bridge─────────→ 시뮬레이션 노드들
```

---

## 2. 완료된 작업 현황

| Phase | 내용 | 상태 |
|-------|------|------|
| **2** | 알고리즘 단위 테스트 (Complementary Filter, Kalman, PID, Navigation) | ✅ 완료 |
| **2.5** | Acados NMPC 도입 (OCP 정의 → C Solver 자동 생성 → C++ 래퍼 연동) | ✅ 완료 |
| **3** | micro-ROS 펌웨어 빌드 검증 (Zephyr SDK 0.16.8, nucleo_h7a3zi_q) | ✅ 완료 |
| **4 (FW)** | 펌웨어 하드웨어 I/F 구현 (GPIO Stepper / I2C AS5600, BNO055) | ✅ 완료 |
| **4 (HW)** | RPi4B 세팅 스크립트 (Acados 포함) | ✅ 스크립트 완성 |
| **6** | Gazebo Fortress 마이그레이션 + sim_motor_bridge_node | ✅ 완료 |
| **7** | GCS 웹 대시보드 (컴패스, RSSI 바, rosbridge) | ✅ 완료 |
| **8** | GitHub Actions CI/CD + GHCR Docker 배포 | ✅ 완료 |
| **5** | 하드웨어 통합 현장 테스트 | ⏳ 장비 연결 후 실행 필요 |

---

## 3. 핵심 파일 목록

```
antenna_tracker_ros/
├── docker/Dockerfile                         ← ROS2 Humble + Acados 통합 환경
├── .github/workflows/ci_cd.yml               ← colcon build/test + GHCR 자동 배포
├── scripts/
│   ├── install_preempt_rt.sh                 ← RPi4B RT 커널 빌드 및 설치
│   ├── setup_ros2_humble.sh                  ← ROS2 Humble + Acados + DDS 설정
│   ├── setup_micro_ros.sh                    ← micro-ROS agent 워크스페이스 구성
│   └── setup_can.sh                          ← CAN0 인터페이스 500kbps 활성화
├── firmware/                                 ← STM32H7 Zephyr micro-ROS 펌웨어
│   ├── COLCON_IGNORE                         ← colcon 스캔 제외
│   ├── prj.conf                              ← Zephyr 커널 설정 (Newlib, CAN 등)
│   ├── app.overlay                           ← GPIO(Stepper) / I2C(AS5600, BNO055) 핀 매핑
│   └── src/main.c                            ← micro-ROS init, 구독/발행, 10kHz 타이머
├── scripts/
│   ├── generate_nmpc_solver.py               ← Acados OCP 정의 → C 코드 자동 생성
│   └── c_generated_code/                     ← 생성된 NMPC C solver
└── src/
    ├── antenna_tracker_msgs/                 ← 커스텀 메시지/서비스/액션
    ├── antenna_tracker_controller/
    │   ├── src/mpc_controller.cpp            ← Acados C solver C++ 래퍼
    │   ├── src/controller_node.cpp           ← 100Hz NMPC 루프
    │   ├── src/sensor_fusion_node.cpp        ← Complementary + Kalman
    │   ├── src/navigation_node.cpp           ← Haversine 방위각/고각
    │   └── src/state_machine_node.cpp        ← AUTO/MANUAL/STANDBY/EMERGENCY
    ├── antenna_tracker_hardware/
    │   └── src/can_bridge_node.cpp           ← SocketCAN → /antenna/target_gps
    ├── antenna_tracker_simulation/
    │   ├── urdf/antenna_tracker.urdf.xacro   ← Gazebo Fortress 플러그인 (gz-sim-*)
    │   ├── worlds/antenna_tracker.world      ← 서울 좌표 (37.5665N, 126.9780E)
    │   ├── launch/sim.launch.py              ← ros_gz_sim + ros_gz_bridge + motor bridge
    │   └── src/sim_motor_bridge_node.cpp     ← MotorCommand Hz → Joint cmd_vel 변환
    ├── antenna_tracker_bringup/
    │   ├── launch/hardware.launch.py         ← 전체 하드웨어 런치 (micro_ros_agent 포함)
    │   ├── launch/web.launch.py              ← rosbridge :9090 + HTTP :8080
    │   └── config/hardware_params.yaml       ← NMPC/PID/필터 파라미터
    └── antenna_tracker_web/
        ├── web/index.html                    ← GCS 대시보드 (Inter/Roboto Mono 디자인)
        ├── web/app.js                        ← Canvas 컴패스, RSSI 바, roslib 클라이언트
        └── src/web_server_node.py            ← Python HTTP 정적 파일 서버 노드
```

---

## 4. ROS2 토픽/서비스/액션

```
Publishers (STM32 → RPi4B):
  /imu/raw                     sensor_msgs/Imu              100Hz
  /magnetic_field              sensor_msgs/MagneticField    100Hz
  /gps/fix                     sensor_msgs/NavSatFix        1Hz
  /antenna/encoder_feedback    EncoderFeedback              100Hz

Publishers (RPi4B 내부):
  /antenna/motor_cmd           MotorCommand                 100Hz
  /antenna/state               AntennaState                 10Hz
  /antenna/target_gps          TargetGPS                    (CAN 수신 시)
  /antenna/diagnostics         TrackerDiagnostics           1Hz

Services:
  /antenna/set_mode            SetMode
  /antenna/set_manual_target   SetManualTarget
  /antenna/get_status          GetStatus

Actions:
  /antenna/track_target        TrackTarget
```

---

## 5. Phase 5 — 하드웨어 통합 테스트 (남은 작업)

> 실제 장비를 연결한 후 아래 순서로 진행.

### 5-1. RPi4B 환경 설정 (1회)

```bash
cd ~/antenna_tracker_ros/scripts
bash setup_ros2_humble.sh          # ROS2 + Acados (30~60분)
bash install_preempt_rt.sh         # PREEMPT_RT 커널 빌드 (~2시간) → reboot
bash setup_micro_ros.sh            # micro-ROS agent 빌드
bash setup_can.sh                  # can0 (ESP32 LoRa) 활성화
```

### 5-2. ROS2 워크스페이스 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5-3. STM32 펌웨어 플래시

```bash
# 개발 PC에서 (Zephyr SDK 필요)
cd antenna_tracker_ros/firmware
west build -b nucleo_h7a3zi_q
west flash
```

### 5-4. 하드웨어 실행

```bash
# 하드웨어 전체 런치
ros2 launch antenna_tracker_bringup hardware.launch.py

# 별도 터미널 — 웹 대시보드
ros2 launch antenna_tracker_bringup web.launch.py
# 브라우저: http://<RPi4B-IP>:8080
```

### 5-5. 검증 체크리스트

| 항목 | 검증 명령 | 목표 |
|------|-----------|------|
| IMU 100Hz | `ros2 topic hz /imu/raw` | ≥ 95Hz |
| 인코더 100Hz | `ros2 topic hz /antenna/encoder_feedback` | ≥ 95Hz |
| 제어 루프 지터 | `sudo cyclictest -l100000 -m -Sp90 -i200 -h400 -q` | Max < 2ms |
| CAN 수신 | `candump can0` | ESP32 패킷 확인 |
| 비상정지 | `ros2 service call /antenna/set_mode … {mode:3}` | 즉시 정지 |

---

## 6. 미해결 이슈 및 주의사항

1. **Hall 센서 핀 매핑** — `app.overlay`의 Dual-Bus 설정(I2C1/I2C2)과 실제 배선 일치 여부 확인 필수.
2. **BMI270 초기화** — 현재 기본적인 가속도/자이로 읽기만 구현됨. 고정밀 융합을 위해서는 Config Blob 로드가 필요할 수 있음.
3. **GPS 별칭(Alias)** — `app.overlay`에 `gps_uart` alias가 `&uartx`에 정확히 매핑되어야 함.
4. **PREEMPT_RT 필수** — RT 커널 없이는 제어 루프 지터 5~20ms 발생 가능.
5. **MCP2515 SPI-CAN 설정 필수** — RPi4B는 네이티브 CAN 포트가 없어 **MCP2515** SPI-CAN 컨트롤러 모듈을 사용해야 함. `scripts/setup_can.sh`가 `/boot/firmware/config.txt`에 DTB Overlay(`dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25`)를 자동 추가함. 모듈의 **오실레이터 주파수(8/12/16MHz)** 를 실물과 맞춰 `oscillator=` 값을 수정할 것. 배선: VCC→3.3V, GND, SCK→GPIO11, MOSI→GPIO10, MISO→GPIO9, CS→GPIO8(CE0), INT→GPIO25.

---

## 7. 빠른 시작 명령 요약

```bash
# 시뮬레이션 (장비 불필요)
ros2 launch antenna_tracker_bringup sim.launch.py

# 하드웨어 (STM32 + RPi4B 연결 필요)
ros2 launch antenna_tracker_bringup hardware.launch.py

# 웹 대시보드
ros2 launch antenna_tracker_bringup web.launch.py
# → http://<RPi4B-IP>:8080

# RT 우선순위로 controller_node 실행
sudo chrt -f 80 taskset -c 2,3 ros2 run antenna_tracker_controller controller_node
```

---

## 8. 참고 파일 경로

| 목적 | 경로 |
|------|------|
| 원본 PID 수치 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/control.h` |
| 원본 Kalman 코드 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/sensor_fusion.c` |
| 원본 CAN 프로토콜 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/protocol.h` |
| 원본 config 수치 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/config.h` |
| NMPC OCP 정의 | `scripts/generate_nmpc_solver.py` |
| Acados 생성 코드 | `scripts/c_generated_code/` |
| GitHub Actions | `.github/workflows/ci_cd.yml` |
| Docker 환경 | `docker/Dockerfile` |
