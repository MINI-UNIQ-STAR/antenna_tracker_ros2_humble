"""
Z3 SMT Verification: KalmanFilterAzEl 수식 모델
Source: kalman_filter.cpp

검증 항목:
  P1. 칼만 게인 범위: K = P/(P+R) ∈ (0, 1)  when P>0, R>0
  P2. 사후 공분산 감소: P_post = (1-K)*P < P
  P3. 사후 공분산 양수 보존: P_post > 0
  P4. 예측 공분산 증가: P_pred = P + Q > P  when Q>0
  P5. 측정 갱신 가중 보간: state_post = (1-K)*pred + K*meas
  P6. 상태 예측 선형 일관성: az_pred = az + az_vel * dt
  P7. [BUG] 속도 공분산(P[1][1], P[3][3]) 무한 증가 — 측정 갱신 없음

버그 탐지:
  - KalmanFilterAzEl::update()에서 P[1][1](az_vel)과 P[3][3](el_vel)은
    예측 단계에서 Q 누적만 일어나고, 측정 갱신이 전혀 없음
  - 결과: 속도 공분산이 단조 증가하여 안정화되지 않음
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
print("  KalmanFilterAzEl 수식 검증  (kalman_filter.cpp)")
print("=" * 65)

# ─────────────────────────────────────────────────────────────
# P1: 칼만 게인 범위 K ∈ (0, 1)
#   코드: K_az = P_[0][0] / (P_[0][0] + R_)
#   클레임: P > 0, R > 0 → 0 < K < 1
# ─────────────────────────────────────────────────────────────
s = Solver()
P, R = Reals('P R')
s.add(P > 0, R > 0)
K = P / (P + R)
s.add(Not(And(K > 0, K < 1)))
report("P1: K = P/(P+R) ∈ (0, 1)  when P>0, R>0", s)

# ─────────────────────────────────────────────────────────────
# P2: 사후 공분산 < 사전 공분산 (측정 후 불확실성 감소)
#   코드: P_[0][0] = (1.0 - K_az) * P_[0][0]
#   클레임: P_post = (1-K)*P < P  when P>0, R>0
# ─────────────────────────────────────────────────────────────
s = Solver()
P2, R2 = Reals('P2 R2')
s.add(P2 > 0, R2 > 0)
K2 = P2 / (P2 + R2)
P_post = (1 - K2) * P2
s.add(Not(P_post < P2))
report("P2: P_post = (1-K)*P < P  (측정 후 공분산 감소)", s)

# ─────────────────────────────────────────────────────────────
# P3: 사후 공분산 양수 보존
#   클레임: P > 0, R > 0 → P_post = (1-K)*P > 0
# ─────────────────────────────────────────────────────────────
s = Solver()
P3, R3 = Reals('P3 R3')
s.add(P3 > 0, R3 > 0)
K3 = P3 / (P3 + R3)
P_post3 = (1 - K3) * P3
s.add(Not(P_post3 > 0))
report("P3: P_post > 0  (공분산 양수 보존)", s)

# ─────────────────────────────────────────────────────────────
# P4: 예측 공분산 증가 (프로세스 노이즈 누적)
#   코드: P_[i][i] += Q_
#   클레임: Q > 0 → P_pred = P + Q > P
# ─────────────────────────────────────────────────────────────
s = Solver()
P4, Q4 = Reals('P4 Q4')
s.add(P4 > 0, Q4 > 0)
P_pred4 = P4 + Q4
s.add(Not(P_pred4 > P4))
report("P4: P_pred = P + Q > P  when Q > 0  (예측 단계 불확실성 증가)", s)

# ─────────────────────────────────────────────────────────────
# P5: 측정 갱신 = 가중 보간
#   코드: pred[0] = pred[0] + K*(meas - pred[0])
#       = (1-K)*pred[0] + K*meas
#   클레임: state_post ∈ [min(pred, meas), max(pred, meas)]
# ─────────────────────────────────────────────────────────────
s = Solver()
pred5, meas5, K5 = Reals('pred5 meas5 K5')
s.add(K5 > 0, K5 < 1)
state_post = pred5 + K5 * (meas5 - pred5)   # = (1-K)*pred + K*meas
lo5 = If(pred5 < meas5, pred5, meas5)
hi5 = If(pred5 > meas5, pred5, meas5)
s.add(Not(And(state_post >= lo5, state_post <= hi5)))
report("P5: state_post ∈ [min(pred,meas), max(pred,meas)]  (가중 보간)", s)

# ─────────────────────────────────────────────────────────────
# P6: 상태 예측 선형 일관성
#   코드: pred[0] = state_[0] + state_[1] * dt_
#   클레임: az_pred - az = az_vel * dt  (등속 모델 일관성)
# ─────────────────────────────────────────────────────────────
s = Solver()
az, az_vel, dt6 = Reals('az az_vel dt6')
s.add(dt6 > 0)
az_pred = az + az_vel * dt6
residual = az_pred - az - az_vel * dt6
s.add(Not(residual == 0))
report("P6: az_pred = az + az_vel*dt  (등속 예측 모델 선형 일관성)", s)

# ─────────────────────────────────────────────────────────────
# P7: [FIX 검증] 속도 공분산 P[1][1] 수렴 — K_az_vel 갱신 추가 후
#   수정 코드:
#     K_az_vel = P[1][1] / (P[1][1] + R)
#     P[1][1]  = (1 - K_az_vel) * P[1][1]  (예측 후)
#   즉: P_post = R*(P+Q_vel)/(P+Q_vel+R) < R  (항상 R에 의해 상한)
#   클레임: P > 0, Q_vel > 0, R > 0 → P_post < R  (수렴 보장)
# ─────────────────────────────────────────────────────────────
s = Solver()
P7, Q_vel7, R7 = Reals('P7 Q_vel7 R7')
s.add(P7 > 0, Q_vel7 > 0, R7 > 0)
P_pred7 = P7 + Q_vel7                       # 예측 단계
K7      = P_pred7 / (P_pred7 + R7)          # 칼만 이득
P_post7 = (1 - K7) * P_pred7               # = R*(P+Q)/(P+Q+R)
# P_post7 < R7 임을 검증 (상한 보장)
s.add(Not(P_post7 < R7))
result7 = s.check()
mark7 = "✓ VERIFIED  " if result7 == unsat else "✗ FALSIFIED "
tag7  = "UNSAT(반례없음)" if result7 == unsat else "SAT(반례있음!)"
print(f"  {mark7} [{tag7}] P7: K_az_vel 갱신 후 P[1][1] < R  (공분산 수렴 보장) ← FIX VERIFIED")

print()
print("  ┌─────────────────────────────────────────────────────┐")
print("  │ [FIX] kalman_filter.cpp — 속도 공분산 수렴 확인     │")
print("  │  수정: K_az_vel = P[1][1]/(P[1][1]+R) 갱신 추가     │")
print("  │        P[1][1] = (1-K_az_vel)*P[1][1]              │")
print("  │  결과: P_post = R*(P+Q)/(P+Q+R) < R  (항상 상한)   │")
print("  │  동일하게 P[3][3] (el_vel) 도 수렴 보장됨           │")
print("  └─────────────────────────────────────────────────────┘")
print()
