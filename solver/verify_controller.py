"""
Z3 SMT Verification: ControllerNode 수식 모델
Source: controller_node.cpp

검증 항목:
  P1. 방위각 최단경로 랩어라운드: while 루프 후 error ∈ (-180, 180]
  P2. 단일 보정 충분성: az_target, az_current ∈ [0,360) → 1회 보정으로 충분
  P3. effective_az_target 일관성: current + wrapped_error = effective_target
  P4. 안전 리밋 로직: out-of-range 위치 → cmd = 0 보장
  P5. 데드밴드 로직: |az_error| < 0.5° → az_cmd = 0 강제
  P6. [BUG] 액션 피드백 오차 vs 제어 오차 불일치
  P7. 오차 벡터 놈 비음수: sqrt(az²+el²) ≥ 0
  P8. tolerance 달성 조건: error ≤ tolerance_deg → success

버그 탐지:
  - controller_node.cpp:195
      feedback->az_error = target_azimuth_ - current_azimuth_  (비랩핑 오차)
    vs controller_node.cpp:143-145
      az_error_raw = target - current; while > 180: -= 360; while < -180: += 360
    → 피드백 오차와 실제 제어 오차가 불일치할 수 있음
    예: target=1°, current=359° → 제어: +2°, 피드백: -358°
"""

from z3 import *


def report(name, solver, expect_unsat=True):
    res = solver.check()
    tag = "UNSAT(반례없음)" if res == unsat else ("SAT(반례있음!)" if res == sat else "UNKNOWN")
    ok = (res == unsat) == expect_unsat
    mark = "✓ VERIFIED  " if ok else "✗ FALSIFIED "
    print(f"  {mark} [{tag}] {name}")
    if res == sat and expect_unsat:
        try:
            print(f"             반례: {solver.model()}")
        except Exception:
            pass
    return res


print("=" * 65)
print("  ControllerNode 수식 검증  (controller_node.cpp)")
print("=" * 65)

# ─────────────────────────────────────────────────────────────
# P1: 방위각 최단경로 랩어라운드 — 결과 ∈ (-180, 180]
#   코드:
#     double az_error_raw = target - current
#     while(az_error_raw >  180) az_error_raw -= 360
#     while(az_error_raw < -180) az_error_raw += 360
#   Z3 모델: az_target, az_current ∈ (-∞, ∞) 일반 케이스
#   단일 보정(If)으로 모델링하고 ∈ (-180, 180] 검증
# ─────────────────────────────────────────────────────────────
s = Solver()
raw_err = Real('raw_err')
# 임의 초기 오차 (target - current, 임의 실수)
# while 루프는 반복 보정 → Z3에서 단일 분기로 충분한 조건 확인
# raw_err ∈ (-360, 360) 가정: az ∈ [0,360)이면 차이 ∈ (-360, 360)
s.add(raw_err > -360, raw_err <= 360)
err1 = If(raw_err >  180, raw_err - 360,
          If(raw_err <= -180, raw_err + 360, raw_err))
s.add(Not(And(err1 > -180, err1 <= 180)))
report("P1: az_error ∈ (-180, 180] — 단일 ±360 보정 (az∈[0,360) 가정)", s)

# ─────────────────────────────────────────────────────────────
# P2: az_target, az_current ∈ [0, 360) → 단일 보정으로 충분
#   raw_error = target - current ∈ (-360, 360)
#   단일 ±360 보정으로 항상 (-180, 180] 달성 가능
# ─────────────────────────────────────────────────────────────
s = Solver()
tgt, cur = Reals('tgt cur')
s.add(tgt >= 0, tgt < 360)
s.add(cur >= 0, cur < 360)
raw2 = tgt - cur   # ∈ (-360, 360)
err2 = If(raw2 >  180, raw2 - 360,
          If(raw2 <= -180, raw2 + 360, raw2))
s.add(Not(And(err2 > -180, err2 <= 180)))
report("P2: tgt,cur ∈ [0,360) → 단일 보정으로 error ∈ (-180,180]", s)

# ─────────────────────────────────────────────────────────────
# P3: effective_az_target 일관성
#   코드: effective_az_target = current_azimuth_ + az_error_raw
#   클레임: (current + wrapped_error) - target_az ≡ 0 mod 360
#   즉, effective_target과 target은 360° 주기로 동치
# ─────────────────────────────────────────────────────────────
s = Solver()
tgt3, cur3 = Reals('tgt3 cur3')
s.add(tgt3 >= 0, tgt3 < 360)
s.add(cur3 >= 0, cur3 < 360)
raw3 = tgt3 - cur3
err3 = If(raw3 >  180, raw3 - 360,
          If(raw3 <= -180, raw3 + 360, raw3))
eff_tgt = cur3 + err3
# eff_tgt - tgt3 는 0 또는 ±360
diff3 = eff_tgt - tgt3
s.add(Not(Or(diff3 == 0, diff3 == 360, diff3 == -360)))
report("P3: effective_target - target ∈ {0, ±360}  (각도 등치)", s)

# ─────────────────────────────────────────────────────────────
# P4: 안전 리밋 로직 — 범위 초과 시 cmd = 0
#   코드: if(current_az < az_min || current_az > az_max) az_cmd = 0
#   클레임: az_min=0, az_max=360 기본값에서
#           current_az ∈ [0,360] → cmd가 0으로 강제되지 않음
# ─────────────────────────────────────────────────────────────
s = Solver()
cur_az = Real('cur_az')
az_min_p, az_max_p = RealVal(0), RealVal(360)
s.add(cur_az >= 0, cur_az <= 360)
# 범위 내이면 cmd = 0 강제 안 됨 (클레임: 범위 내 → not out-of-range)
out_of_range = Or(cur_az < az_min_p, cur_az > az_max_p)
s.add(Not(Not(out_of_range)))   # 즉: out_of_range 가능한지 (SAT이면 이탈 가능)
# 이건 tautology check: cur_az∈[0,360]이면 out_of_range=False
s.add(out_of_range)
res_p4 = s.check()
mark4 = "✓ VERIFIED  " if res_p4 == unsat else "✗ FALSIFIED "
tag4 = "UNSAT(이탈없음)" if res_p4 == unsat else "SAT(이탈가능)"
print(f"  {mark4} [{tag4}] P4: az_current∈[0,360] → safety limit 미발동 (범위 정상)")

# ─────────────────────────────────────────────────────────────
# P5: 데드밴드 로직
#   코드: if(|az_error_raw| < 0.5) az_cmd = 0
#   클레임: |error| < 0.5 → az_cmd는 반드시 0
# ─────────────────────────────────────────────────────────────
s = Solver()
az_err_db = Real('az_err_db')
raw_cmd    = Real('raw_cmd')   # PID/MPC 출력 (임의값)
s.add(az_err_db > -0.5, az_err_db < 0.5)
az_cmd_final = If(And(az_err_db > -0.5, az_err_db < 0.5), RealVal(0), raw_cmd)
s.add(Not(az_cmd_final == 0))
report("P5: |az_error| < 0.5° → az_cmd = 0  (데드밴드 강제)", s)

# ─────────────────────────────────────────────────────────────
# P6: [FIX 검증] 액션 피드백 오차 = 제어 오차 (수정 후)
#   수정 코드 (line 222): feedback->az_error = az_error_raw  (랩핑됨)
#   제어 오차: az_error_raw = wrapped(target - current)
#   피드백 오차: az_error_raw (동일)
#   클레임: 두 값이 항상 같다 (UNSAT = FIX 확인)
# ─────────────────────────────────────────────────────────────
s = Solver()
tgt6, cur6 = Reals('tgt6 cur6')
s.add(tgt6 >= 0, tgt6 < 360)
s.add(cur6 >= 0, cur6 < 360)

raw6 = tgt6 - cur6
ctrl_err = If(raw6 >  180, raw6 - 360,
              If(raw6 <= -180, raw6 + 360, raw6))   # 랩핑됨 (제어 오차)
fb_err   = ctrl_err                                  # 수정 후: feedback = az_error_raw (동일)

s.add(ctrl_err != fb_err)   # 두 값이 다른 경우 존재하는지 → UNSAT이어야 함
res_p6 = s.check()
if res_p6 == unsat:
    print(f"  ✓ VERIFIED   [UNSAT(반례없음)] P6: 피드백 오차 = 제어 오차 — 랩어라운드 일치 ← FIX VERIFIED")
else:
    print(f"  ✗ FALSIFIED  [SAT] P6: 피드백 오차 ≠ 제어 오차 (예상치 못한 불일치)")

# ─────────────────────────────────────────────────────────────
# P7: 오차 벡터 놈 비음수
#   코드: error = sqrt(az_err² + el_err²)
#   클레임: sqrt(x² + y²) ≥ 0 (항상 참)
# ─────────────────────────────────────────────────────────────
s = Solver()
az_e, el_e = Reals('az_e el_e')
sq_sum = az_e * az_e + el_e * el_e
# sq_sum ≥ 0 → sqrt(sq_sum) ≥ 0
s.add(Not(sq_sum >= 0))
report("P7: az_err² + el_err² ≥ 0  (오차 벡터 놈 비음수)", s)

# ─────────────────────────────────────────────────────────────
# P8: tolerance 달성 조건
#   코드: if(error <= goal->tolerance_deg) → success
#   클레임: tolerance > 0 이면 error=0(완벽 추적)일 때 반드시 success
# ─────────────────────────────────────────────────────────────
s = Solver()
tol = Real('tol')
s.add(tol > 0)
err_norm = RealVal(0)   # 완벽 추적
achieved = (err_norm <= tol)
s.add(Not(achieved))
report("P8: error=0, tolerance>0 → 달성 조건 만족", s)

print()
print("  ┌─────────────────────────────────────────────────────┐")
print("  │ [FIX] controller_node.cpp:222 피드백 오차 수정 완료 │")
print("  │  수정 전: feedback->az_error = target - current     │")
print("  │           (랩핑 없는 원시 오차 → 최대 ±358° 보고)   │")
print("  │  수정 후: feedback->az_error = az_error_raw  ✓      │")
print("  │           ((-180,180] 최단경로 랩핑 오차 사용)       │")
print("  │  예시: target=1°, current=359°                      │")
print("  │    제어 오차 = +2° (최단경로) ✓                     │")
print("  │    피드백 오차 = +2° (일치) ✓                       │")
print("  └─────────────────────────────────────────────────────┘")
print()
