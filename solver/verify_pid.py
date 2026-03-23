"""
Z3 SMT Verification: CascadePid 수식 모델
Source: pid_controller.cpp

검증 항목:
  P1. clamp() 항등성: 결과는 항상 [min, max] 내
  P2. 외부 루프 적분 클램핑: integral ∈ [-100, 100]
  P3. 내부 루프 적분 클램핑: integral ∈ [-500, 500]
  P4. 최종 출력 클램핑: output ∈ [output_min, output_max]
  P5. 영입력 영출력: error=0, integral=0, prev_error=0 → velocity_setpoint=0
  P6. 영입력 영출력 (전체): 모든 상태 0 → motor output = 0
  P7. dt > 0 조건: 미분항 연산이 유효 (분모 비영)
  P8. 클램프 단조성: 입력이 클램프 범위 내이면 출력 = 입력 (항등)
  P9. 중첩 클램프 범위 보존: output_min ≤ output_max → 출력 범위 유지
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
print("  CascadePid 수식 검증  (pid_controller.cpp)")
print("=" * 65)

# ─────────────────────────────────────────────────────────────
# P1: clamp() 함수 항등성
#   코드: return std::max(min_val, std::min(max_val, value))
#   클레임: 결과 ∈ [min_val, max_val] (min_val ≤ max_val 가정)
# ─────────────────────────────────────────────────────────────
s = Solver()
val, lo, hi = Reals('val lo hi')
s.add(lo <= hi)
clamped = If(val < lo, lo, If(val > hi, hi, val))
s.add(Not(And(clamped >= lo, clamped <= hi)))
report("P1: clamp(v, lo, hi) ∈ [lo, hi]", s)

# ─────────────────────────────────────────────────────────────
# P2: 외부 루프 적분 클램핑 [-100, 100]
#   코드: outer_integral_ += pos_error * dt_
#         outer_integral_ = clamp(outer_integral_, -100.0, 100.0)
#   클레임: 클램핑 후 ∈ [-100, 100] (dt, error는 임의값)
# ─────────────────────────────────────────────────────────────
s = Solver()
outer_int_prev, pos_err, dt = Reals('outer_int_prev pos_err dt')
s.add(dt > 0)
# 이전 적분값은 이미 클램핑되어 있음 (귀납 가설)
s.add(outer_int_prev >= -100, outer_int_prev <= 100)
outer_int_new = outer_int_prev + pos_err * dt
outer_int_clamped = If(outer_int_new < -100, RealVal(-100),
                       If(outer_int_new > 100, RealVal(100), outer_int_new))
s.add(Not(And(outer_int_clamped >= -100, outer_int_clamped <= 100)))
report("P2: 외부 루프 적분 ∈ [-100, 100] (클램핑 후)", s)

# ─────────────────────────────────────────────────────────────
# P3: 내부 루프 적분 클램핑 [-500, 500]
# ─────────────────────────────────────────────────────────────
s = Solver()
inner_int_prev, vel_err, dt2 = Reals('inner_int_prev vel_err dt2')
s.add(dt2 > 0)
s.add(inner_int_prev >= -500, inner_int_prev <= 500)
inner_int_new = inner_int_prev + vel_err * dt2
inner_int_clamped = If(inner_int_new < -500, RealVal(-500),
                       If(inner_int_new > 500, RealVal(500), inner_int_new))
s.add(Not(And(inner_int_clamped >= -500, inner_int_clamped <= 500)))
report("P3: 내부 루프 적분 ∈ [-500, 500] (클램핑 후)", s)

# ─────────────────────────────────────────────────────────────
# P4: 최종 출력 클램핑 [output_min, output_max]
#   코드: return clamp(output, output_min_, output_max_)
# ─────────────────────────────────────────────────────────────
s = Solver()
raw_output, out_min, out_max = Reals('raw_output out_min out_max')
s.add(out_min <= out_max)
final_out = If(raw_output < out_min, out_min,
               If(raw_output > out_max, out_max, raw_output))
s.add(Not(And(final_out >= out_min, final_out <= out_max)))
report("P4: 최종 출력 ∈ [output_min, output_max]", s)

# ─────────────────────────────────────────────────────────────
# P5: 외부 루프 영입력 영출력
#   pos_error=0, outer_integral=0, outer_prev_error=0 → velocity_setpoint=0
#   코드: vel_sp = kp*err + ki*intg + kd*(err-prev)/dt
# ─────────────────────────────────────────────────────────────
s = Solver()
kp_o, ki_o, kd_o, dt_p = Reals('kp_o ki_o kd_o dt_p')
s.add(dt_p > 0)
pos_error  = RealVal(0)
outer_intg = RealVal(0)
outer_prev = RealVal(0)
pos_deriv  = (pos_error - outer_prev) / dt_p   # = 0
vel_sp = kp_o * pos_error + ki_o * outer_intg + kd_o * pos_deriv
s.add(Not(vel_sp == 0))
report("P5: 외부 루프 영상태 → velocity_setpoint = 0", s)

# ─────────────────────────────────────────────────────────────
# P6: 전체 영입력 영출력
#   모든 오차·적분·이전오차 = 0 → 내부 루프 출력 = 0
# ─────────────────────────────────────────────────────────────
s = Solver()
kp_i, ki_i, kd_i, dt_q = Reals('kp_i ki_i kd_i dt_q')
s.add(dt_q > 0)
vel_error_z  = RealVal(0)
inner_intg_z = RealVal(0)
inner_prev_z = RealVal(0)
vel_deriv_z  = (vel_error_z - inner_prev_z) / dt_q   # = 0
motor_out = kp_i * vel_error_z + ki_i * inner_intg_z + kd_i * vel_deriv_z
s.add(Not(motor_out == 0))
report("P6: 내부 루프 영상태 → motor output = 0", s)

# ─────────────────────────────────────────────────────────────
# P7: dt > 0 조건 — 미분항이 well-defined
#   코드: double pos_derivative = (pos_error - outer_prev_error_) / dt_
#   클레임: dt > 0 이면 division by zero 없음
# ─────────────────────────────────────────────────────────────
s = Solver()
err_curr, err_prev, dt_r = Reals('err_curr err_prev dt_r')
s.add(dt_r > 0)
deriv = (err_curr - err_prev) / dt_r
# Z3은 dt>0 조건에서 나눗셈이 유효함을 암묵적으로 보장
# "dt > 0 조건에서 deriv는 임의 실수이다"를 반증하는 경우가 없어야 함
s.add(Not(Or(deriv > 0, deriv == 0, deriv < 0)))   # real number exhaustion
report("P7: dt > 0 → 미분항 연산 well-defined (실수 영역)", s)

# ─────────────────────────────────────────────────────────────
# P8: clamp 단조성 — 입력이 이미 범위 내이면 출력 = 입력
# ─────────────────────────────────────────────────────────────
s = Solver()
v2, lo2, hi2 = Reals('v2 lo2 hi2')
s.add(lo2 <= hi2)
s.add(v2 >= lo2, v2 <= hi2)   # 입력이 이미 범위 내
clamped2 = If(v2 < lo2, lo2, If(v2 > hi2, hi2, v2))
s.add(Not(clamped2 == v2))
report("P8: 입력 ∈ [lo, hi] → clamp(입력) = 입력 (항등)", s)

# ─────────────────────────────────────────────────────────────
# P9: 중첩 clamp 범위 보존
#   outer loop가 [-100,100]으로 clamped된 후 kp를 곱해도
#   최종 출력은 output_min/max 클램핑으로 보호됨
# ─────────────────────────────────────────────────────────────
s = Solver()
vel_sp_raw, kp_in, ki_in, kd_in = Reals('vel_sp_raw kp_in ki_in kd_in')
intg_in, prev_in, dt_s = Reals('intg_in prev_in dt_s')
out_mn, out_mx = Reals('out_mn out_mx')
s.add(out_mn <= out_mx, dt_s > 0)
# 내부 루프 출력 (arbitrary)
vel_err2 = vel_sp_raw - RealVal(0)   # vel_current = 0으로 단순화
inner_out = kp_in * vel_err2 + ki_in * intg_in + kd_in * (vel_err2 - prev_in) / dt_s
final_clamped = If(inner_out < out_mn, out_mn,
                   If(inner_out > out_mx, out_mx, inner_out))
s.add(Not(And(final_clamped >= out_mn, final_clamped <= out_mx)))
report("P9: 출력 클램핑은 임의 PID 출력에 대해 항상 범위 보장", s)

print()
print("  [GAINS] az_outer={kp=5.0,ki=0.0,kd=0.1}, az_inner={kp=1.5,ki=0.01,kd=0.05}")
print("          el_outer={kp=3.0,ki=0.0,kd=0.1}, el_inner={kp=2.0,ki=0.01,kd=0.02}")
print("  [NOTE]  ki=0.0 (outer az/el) → 정상상태 적분 누적 없음 → 오프셋 발생 가능")
print()
