"""
Headless round-trip test for the geometric IK solver.

Verifies internal consistency: solve_ik() → FK → compare EE position to target.
Also checks that unreachable targets return None (no motion).
"""
import math

# Must match URDF and ik_node.py
BASE_HEIGHT = 0.09   # base(0.07) + turntable(0.02)
L1 = 0.10
L2 = 0.09
L_GRIP = 0.16        # gripper_base(0.03) + fingers(0.13)

J2_LIMIT = math.pi / 2
J3_LIMIT = math.pi / 2
J4_LIMIT = math.pi / 2


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def solve_ik(x, y, z):
    """Returns (theta1, q2, q3, q4) or None if unreachable."""
    if z < 0:
        return None

    theta1 = math.atan2(y, x)
    r_target = math.sqrt(x * x + y * y)

    r_w = r_target
    h_w = (z + L_GRIP) - BASE_HEIGHT

    d_sq = r_w ** 2 + h_w ** 2
    d = math.sqrt(d_sq)

    if d >= L1 + L2:
        return None
    if d <= abs(L1 - L2):
        return None

    cos_q3 = clamp((d_sq - L1 ** 2 - L2 ** 2) / (2 * L1 * L2), -1.0, 1.0)
    alpha = math.atan2(r_w, h_w)

    best = None
    best_cost = float('inf')

    for q3_val in [math.acos(cos_q3), -math.acos(cos_q3)]:
        beta = math.atan2(L2 * math.sin(q3_val), L1 + L2 * math.cos(q3_val))
        q2_val = alpha - beta
        q4_val = math.pi - q2_val - q3_val

        if abs(q2_val) > J2_LIMIT:
            continue
        if abs(q3_val) > J3_LIMIT:
            continue
        if abs(q4_val) > J4_LIMIT:
            continue

        cost = abs(q2_val) + abs(q3_val) + abs(q4_val)
        if cost < best_cost:
            best_cost = cost
            best = (q2_val, q3_val, q4_val)

    if best is None:
        return None

    q2, q3, q4 = best
    return clamp(theta1, -math.pi, math.pi), q2, q3, q4


def fk(theta1, q2, q3, q4):
    """Forward kinematics — returns EE tip (x, y, z) in world frame."""
    # Arm works in vertical plane at angle theta1 from X axis
    # In the arm plane: horizontal = r, vertical = h
    # URDF convention: 0 = pointing up, Y-axis rotation
    r = (L1 * math.sin(q2)
         + L2 * math.sin(q2 + q3)
         + L_GRIP * math.sin(q2 + q3 + q4))
    h = (BASE_HEIGHT
         + L1 * math.cos(q2)
         + L2 * math.cos(q2 + q3)
         + L_GRIP * math.cos(q2 + q3 + q4))
    x = r * math.cos(theta1)
    y = r * math.sin(theta1)
    z = h
    return x, y, z


def run_tests():
    print("Round-trip IK verification (solve → FK → compare)")
    print(f"Geometry: BASE_HEIGHT={BASE_HEIGHT}, L1={L1}, L2={L2}, L_GRIP={L_GRIP}")
    print(f"Max reach from shoulder: {L1+L2:.3f} m")
    print()

    PASS = "\033[92mPASS\033[0m"
    FAIL = "\033[91mFAIL\033[0m"

    # --- Reachable targets ---
    # Ground-level annulus: ~0.115 m <= r <= ~0.160 m (from base axis)
    reachable = [
        (0.120, 0.000, 0.000),
        (0.140, 0.000, 0.000),
        (0.150, 0.000, 0.000),
        (0.000, 0.150, 0.000),   # same reach, rotated 90°
        (0.106, 0.106, 0.000),   # r ≈ 0.150, diagonal
        (0.160, 0.000, 0.000),
    ]

    all_passed = True
    for target in reachable:
        x, y, z = target
        sol = solve_ik(x, y, z)
        if sol is None:
            print(f"  target ({x:.3f}, {y:.3f}, {z:.3f}): {FAIL} — solve returned None (expected reachable)")
            all_passed = False
            continue
        ex, ey, ez = fk(*sol)
        err = math.sqrt((ex - x) ** 2 + (ey - y) ** 2 + (ez - z) ** 2)
        status = PASS if err < 1e-6 else FAIL
        if err >= 1e-6:
            all_passed = False
        print(f"  target ({x:.3f}, {y:.3f}, {z:.3f}): {status}  error={err:.2e} m  "
              f"q=[{math.degrees(sol[0]):.1f}°, {math.degrees(sol[1]):.1f}°, "
              f"{math.degrees(sol[2]):.1f}°, {math.degrees(sol[3]):.1f}°]")

    print()

    # --- Unreachable targets (must return None) ---
    unreachable = [
        (0.50, 0.00, 0.00, "too far from base"),
        (0.05, 0.00, 0.00, "too close, r<0.115"),
        (0.00, 0.00, 0.00, "at base axis, r=0"),
        (0.15, 0.00, -0.10, "below ground"),
        (0.30, 0.30, 0.00, "too far diagonal"),
        (0.17, 0.00, 0.00, "in reach sphere but q4>90°"),
    ]

    for x, y, z, reason in unreachable:
        sol = solve_ik(x, y, z)
        status = PASS if sol is None else FAIL
        if sol is not None:
            all_passed = False
        print(f"  target ({x:.3f}, {y:.3f}, {z:.3f}) [{reason}]: {status}"
              + (f"  — expected None, got {sol}" if sol is not None else "  — correctly returned None"))

    print()
    print("arm_ik_2d round-trip (separate solver, 2D convention):")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    'src', 'manipulator_control'))
    try:
        from manipulator_control import arm_ik_2d as ik2d
        test_2d = [(0.120, 0.00), (0.140, 0.00), (0.150, 0.00)]
        for reach, height in test_2d:
            sol = ik2d.solve(reach, height)
            if sol is None:
                print(f"  2D ({reach:.3f}, {height:.3f}): {FAIL} — returned None (expected reachable)")
                all_passed = False
            else:
                ex, ey = ik2d.forward(sol['shoulder'], sol['elbow'], sol['wrist'])
                err = math.sqrt((ex - reach) ** 2 + (ey - height) ** 2)
                status = PASS if err < 1e-6 else FAIL
                if err >= 1e-6:
                    all_passed = False
                print(f"  2D ({reach:.3f}, {height:.3f}): {status}  error={err:.2e} m")
    except ImportError as e:
        print(f"  (skipped — could not import arm_ik_2d: {e})")

    print()
    print("All tests passed." if all_passed else "SOME TESTS FAILED.")
    return 0 if all_passed else 1


if __name__ == '__main__':
    import sys
    sys.exit(run_tests())
