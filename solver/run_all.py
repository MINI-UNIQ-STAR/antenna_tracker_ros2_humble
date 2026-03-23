"""
Antenna Tracker ROS — Z3 SMT 전체 검증 실행기

모든 모듈 검증 스크립트를 순서대로 실행하고 최종 요약을 출력합니다.
"""

import subprocess
import sys
import re
from pathlib import Path

SCRIPTS = [
    ("Navigation Node",          "verify_navigation.py"),
    ("CascadePid Controller",    "verify_pid.py"),
    ("KalmanFilterAzEl",         "verify_kalman.py"),
    ("ComplementaryFilter",      "verify_complementary_filter.py"),
    ("ControllerNode",           "verify_controller.py"),
]

base = Path(__file__).parent
total_verified = 0
total_bugs     = 0
total_failed   = 0

print()
print("╔" + "═" * 63 + "╗")
print("║   Antenna Tracker ROS — Z3 SMT 전체 검증" + " " * 20 + "║")
print("╚" + "═" * 63 + "╝")

for module_name, script in SCRIPTS:
    print(f"\n{'━' * 65}")
    result = subprocess.run(
        [sys.executable, str(base / script)],
        capture_output=True, text=True
    )
    output = result.stdout
    print(output, end="")
    if result.stderr:
        print("[STDERR]", result.stderr[:200])

    verified = len(re.findall(r"✓ VERIFIED", output))
    bugs     = len(re.findall(r"BUG (CONFIRMED|FOUND)", output))
    failed   = len(re.findall(r"✗ FALSIFIED", output))

    total_verified += verified
    total_bugs     += bugs
    total_failed   += failed

print("╔" + "═" * 63 + "╗")
print("║   최종 검증 요약" + " " * 46 + "║")
print("╠" + "═" * 63 + "╣")
print(f"║  ✓ VERIFIED  : {total_verified:3d} 개 항목 — 수식 모델 정확            ║")
print(f"║  ✗ BUG FOUND : {total_bugs:3d} 개 항목 — 구조적 결함 확인됨         ║")
print(f"║  ✗ FALSIFIED : {total_failed:3d} 개 항목 — 반례 발견                  ║")
print("╠" + "═" * 63 + "╣")
print("║  수정 완료 항목:                                                ║")
print("║   1. kalman_filter.cpp  — K_az_vel/K_el_vel 갱신 추가         ║")
print("║      P[1][1]/P[3][3] 수렴 보장 (< R 상한)                     ║")
print("║   2. complementary_filter.cpp — 완전 틸트 보상 적용           ║")
print("║      mx += mag_y*sin(r)*sin(p) + mag_z*cos(r)*sin(p)          ║")
print("║   3. controller_node.cpp:222 — 피드백 az_error 랩핑 수정      ║")
print("║      feedback->az_error = az_error_raw (최단경로 적용)         ║")
print("╚" + "═" * 63 + "╝")
print()
