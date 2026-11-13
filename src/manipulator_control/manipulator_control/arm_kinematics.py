"""Planar kinematics in the full_arm_controller frame.

Joint angles are physical degrees as the firmware reports and accepts them:
90 deg is the link straight up and a larger angle leans the link outward
(away from the base axis, increasing reach). Each joint's lean adds to the
links before it, which is the same model the firmware gravity term uses:

    a1 = shoulder - 90,  a2 = a1 + elbow - 90,  a3 = a2 + wrist - 90

with a_i the absolute lean of link i from vertical. Reach r is horizontal
distance from the base axis, z is height above the ground. phi is the absolute
tip angle from vertical in degrees: 0 points up, 90 horizontal, 180 down.
"""
from math import acos, atan2, cos, degrees, hypot, radians, sin

BASE_Z = 0.09   # shoulder axis above the ground (m)
L1 = 0.10       # shoulder -> elbow
L2 = 0.09       # elbow -> wrist
L3 = 0.16       # wrist -> gripper tip

# Firmware target limits (TARGET_MIN_DEG / TARGET_MAX_DEG); the wrist is kept
# below 170 deg because its potentiometer nears ADC saturation at 180.
JOINT_LIMITS = ((85.0, 175.0), (85.0, 175.0), (85.0, 170.0))
GROUND_CLEARANCE = 0.0  # no joint or the tip may go below this height (m)


def forward_points(q):
    """Base, elbow, wrist and tip (r, z) for physical joint angles q (deg)."""
    pts = [(0.0, BASE_Z)]
    lean = 0.0
    for angle, length in zip(q, (L1, L2, L3)):
        lean += radians(angle - 90.0)
        r, z = pts[-1]
        pts.append((r + length * sin(lean), z + length * cos(lean)))
    return pts


def forward(q):
    """Tip (r, z, phi_deg) for physical joint angles q (deg)."""
    r, z = forward_points(q)[-1]
    return r, z, sum(q) - 270.0


def within_limits(q):
    return all(lo <= a <= hi for a, (lo, hi) in zip(q, JOINT_LIMITS))


def clears_ground(q):
    return all(z >= GROUND_CLEARANCE for _, z in forward_points(q)[1:])


def _solutions(r, z, phi):
    """Both elbow branches for tip (r, z) at tip angle phi (deg)."""
    p = radians(phi)
    wr, wz = r - L3 * sin(p), z - BASE_Z - L3 * cos(p)
    d = hypot(wr, wz)
    c = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if not -1.0 <= c <= 1.0:
        return []
    out = []
    for e in (acos(c), -acos(c)):
        a1 = atan2(wr, wz) - atan2(L2 * sin(e), L1 + L2 * cos(e))
        a1d, ed = degrees(a1), degrees(e)
        out.append((90.0 + a1d, 90.0 + ed, 90.0 + phi - a1d - ed))
    return out


def solve(r, z, phi, prev=None):
    """Joint angles (deg) reaching (r, z) with tip angle phi, or None.

    Picks the valid branch nearest prev (or the first valid one)."""
    valid = [q for q in _solutions(r, z, phi) if within_limits(q) and clears_ground(q)]
    if not valid:
        return None
    if prev is None:
        return valid[0]
    return min(valid, key=lambda q: sum((a - b) ** 2 for a, b in zip(q, prev)))


def solve_auto(r, z, prev=None, phi_hint=None, step=2.0):
    """Like solve(), searching tip angle outward from phi_hint (deg).

    Returns (q, phi) or None. Without a hint the search starts horizontal."""
    start = 90.0 if phi_hint is None else phi_hint
    k = 0
    while True:
        tried = False
        for phi in ((start,) if k == 0 else (start + k * step, start - k * step)):
            if -10.0 <= phi <= 190.0:
                tried = True
                q = solve(r, z, phi, prev)
                if q is not None:
                    return q, phi
        if not tried:
            return None
        k += 1
