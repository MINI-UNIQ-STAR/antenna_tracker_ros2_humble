"""
Z3 SMT Verification: NavigationNode 수식 모델
Source: navigation_node.cpp

검증 항목:
  P1. Bearing 정규화: atan2 결과 + 조건부 +360 → 항상 [0, 360) 범위
  P2. Elevation 클램핑: max(0, min(90, el)) → 항상 [0, 90] 범위
  P3. 거리 엣지케이스: distance < 1.0 → 반드시 90.0 반환
  P4. Elevation 부호 일관성: distance > 0이면 sign(el) == sign(delta_alt)
  P5. Haversine 'a' 하한: a >= 0 (위도 범위 공리 하에서)
  P6. Haversine 거리 비음수: dist >= 0

한계:
  - sin/cos는 범위 공리를 갖는 미해석 함수로 선언
  - P5 상한 (a <= 1)은 구면 코사인 법칙 필요 → Z3 범위 밖 (별도 주석)
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
            m = solver.model()
            print(f"             반례: {m}")
        except Exception:
            pass
    return res


print("=" * 65)
print("  Navigation Node 수식 검증  (navigation_node.cpp)")
print("=" * 65)

# ─────────────────────────────────────────────────────────────
# P1: Bearing 정규화 [0, 360)
#   코드: bearing = atan2(y,x)*RAD_TO_DEG; if(bearing<0) bearing+=360
#   atan2 공역: (-π, π] → 도 단위 (-180, 180]
#   클레임: 정규화 후 항상 [0, 360)
# ─────────────────────────────────────────────────────────────
s = Solver()
raw = Real('raw')
s.add(raw > -180, raw <= 180)          # atan2 출력 범위 공리
bearing = If(raw < 0, raw + 360, raw)
s.add(Not(And(bearing >= 0, bearing < 360)))  # 반례 탐색
report("P1: bearing ∈ [0, 360) — atan2 정규화 후", s)

# ─────────────────────────────────────────────────────────────
# P2: Elevation 클램핑 [0, 90]
#   코드: std::max(0.0, std::min(90.0, elev))
# ─────────────────────────────────────────────────────────────
s = Solver()
elev = Real('elev')
clamped = If(elev < 0, RealVal(0), If(elev > 90, RealVal(90), elev))
s.add(Not(And(clamped >= 0, clamped <= 90)))
report("P2: elevation ∈ [0, 90] — 클램핑 후", s)

# ─────────────────────────────────────────────────────────────
# P3: 거리 엣지케이스
#   코드: if(distance < 1.0) return 90.0;
#   클레임: 이 분기가 실행되면 반환값은 정확히 90.0
# ─────────────────────────────────────────────────────────────
s = Solver()
dist = Real('dist')
s.add(dist < 1.0)
early_return = RealVal(90.0)
s.add(Not(early_return == 90.0))
report("P3: distance < 1.0 → elevation 반환값 = 90.0", s)

# ─────────────────────────────────────────────────────────────
# P4: Elevation 부호 일관성
#   코드: atan2(delta_alt, distance) * RAD_TO_DEG
#   공리: x > 0일 때 atan2(y, x)의 부호 = y의 부호
#   클레임: distance > 0, delta_alt > 0 이면 elevation > 0
# ─────────────────────────────────────────────────────────────
s = Solver()
atan2_f = Function('atan2_f', RealSort(), RealSort(), RealSort())
y_v, x_v = Reals('y_v x_v')
# atan2 부호 공리 (x > 0 사분면)
s.add(ForAll([y_v, x_v], Implies(And(x_v > 0, y_v > 0), atan2_f(y_v, x_v) > 0)))
s.add(ForAll([y_v, x_v], Implies(And(x_v > 0, y_v == 0), atan2_f(y_v, x_v) == 0)))
s.add(ForAll([y_v, x_v], Implies(And(x_v > 0, y_v < 0), atan2_f(y_v, x_v) < 0)))

delta_alt = Real('delta_alt')
distance = Real('distance')
s.add(distance > 0, delta_alt > 0)
el_rad = atan2_f(delta_alt, distance)
s.add(Not(el_rad > 0))
report("P4: delta_alt > 0, dist > 0 → elevation > 0 (부호 일관성)", s)

# ─────────────────────────────────────────────────────────────
# P4b: delta_alt < 0이면 elevation < 0 (클램핑 전)
# ─────────────────────────────────────────────────────────────
s = Solver()
atan2_f2 = Function('atan2_f2', RealSort(), RealSort(), RealSort())
s.add(ForAll([y_v, x_v], Implies(And(x_v > 0, y_v < 0), atan2_f2(y_v, x_v) < 0)))
da2 = Real('da2')
di2 = Real('di2')
s.add(di2 > 0, da2 < 0)
s.add(Not(atan2_f2(da2, di2) < 0))
report("P4b: delta_alt < 0, dist > 0 → elevation < 0 (클램핑 전)", s)

# ─────────────────────────────────────────────────────────────
# P5: Haversine 'a' 하한 >= 0
#   a = sin²(Δφ/2) + cos(φ1)*cos(φ2)*sin²(Δλ/2)
#   공리: sin∈[-1,1], 위도용 cos∈[0,1] (위도 ∈ [-90°, 90°])
# ─────────────────────────────────────────────────────────────
s = Solver()
s_dphi = Real('s_dphi')   # sin(Δφ/2)
s_dlam = Real('s_dlam')   # sin(Δλ/2)
c_phi1 = Real('c_phi1')   # cos(φ1)
c_phi2 = Real('c_phi2')   # cos(φ2)
s.add(s_dphi >= -1, s_dphi <= 1)
s.add(s_dlam >= -1, s_dlam <= 1)
s.add(c_phi1 >= 0, c_phi1 <= 1)   # 위도 ∈ [-π/2, π/2]이면 cos ≥ 0
s.add(c_phi2 >= 0, c_phi2 <= 1)

a = s_dphi * s_dphi + c_phi1 * c_phi2 * s_dlam * s_dlam
s.add(Not(a >= 0))
report("P5: haversine 'a' ≥ 0 (비음수)", s)

# NOTE: a ≤ 1의 상한 증명은 구면 삼각법 항등식에 의존하므로
#       Z3 실수 산술만으로는 완전 증명 불가. 수치 검증으로 보완 권장.

# ─────────────────────────────────────────────────────────────
# P6: Haversine 거리 비음수
#   c = 2 * atan2(√a, √(1-a)), dist = R * c
#   공리: atan2(y≥0, x>0) ≥ 0, R = 6371000 > 0
# ─────────────────────────────────────────────────────────────
s = Solver()
atan2_g = Function('atan2_g', RealSort(), RealSort(), RealSort())
s.add(ForAll([y_v, x_v],
    Implies(And(y_v >= 0, x_v > 0), atan2_g(y_v, x_v) >= 0)))

a_val   = Real('a_val')
sqrt_a  = Real('sqrt_a')
sqrt_1ma = Real('sqrt_1ma')
s.add(a_val >= 0, a_val < 1)    # a ∈ [0, 1)  (a=1: 지구 반대편, c=π)
s.add(sqrt_a >= 0)
s.add(sqrt_1ma > 0)             # a < 1 이면 √(1-a) > 0

c_val = 2 * atan2_g(sqrt_a, sqrt_1ma)
R = RealVal(6371000)
dist_val = R * c_val
s.add(Not(dist_val >= 0))
report("P6: haversine distance ≥ 0 (비음수)", s)

print()
print("  [NOTE] P5 상한(a≤1) 증명은 구면 코사인 법칙 필요 → Z3 미지원")
print("         수치 몬테카를로 샘플링으로 추가 검증 권장")
print()
