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
from math import acos, atan2, ceil, cos, degrees, hypot, isfinite, radians, sin

BASE_Z = 0.09   # shoulder axis above the ground (m)
L1 = 0.10       # shoulder -> elbow
L2 = 0.09       # elbow -> wrist
L3 = 0.15       # wrist -> gripper tip (measured on the arm)

# Firmware target limits (TARGET_MIN_DEG / TARGET_MAX_DEG): 10 deg inside each
# calibrated end at 0 and 180.
JOINT_LIMITS = ((10.0, 170.0), (10.0, 170.0), (10.0, 170.0))
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
    """Distinct elbow branches, using equivalent physical joint angles."""
    phi = (phi + 180.0) % 360.0 - 180.0
    p = radians(phi)
    wr, wz = r - L3 * sin(p), z - BASE_Z - L3 * cos(p)
    d = hypot(wr, wz)
    c = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if c < -1.0 - 1e-12 or c > 1.0 + 1e-12:
        return []
    e_abs = acos(max(-1.0, min(1.0, c)))
    out = []
    for e in ((e_abs,) if e_abs == 0.0 else (e_abs, -e_abs)):
        a1 = atan2(wr, wz) - atan2(L2 * sin(e), L1 + L2 * cos(e))
        a1d, ed = degrees(a1), degrees(e)
        angles = (90.0 + a1d, 90.0 + ed, 90.0 + phi - a1d - ed)
        physical = []
        for angle, (lo, hi) in zip(angles, JOINT_LIMITS):
            angle += 360.0 * round(((lo + hi) * 0.5 - angle) / 360.0)
            # Analytic solutions at a joint stop can differ by roundoff.
            if lo - 1e-9 <= angle <= hi + 1e-9:
                angle = max(lo, min(hi, angle))
            physical.append(angle)
        q = tuple(physical)
        if not out or any(abs(a - b) > 1e-9 for a, b in zip(q, out[0])):
            out.append(q)
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
    """Return (q, phi) nearest the hinted orientation, or None.

    Without a hint, prefer the tip pointing down (180 degrees). Search both
    directions around the full orientation circle in positive ``step`` degree
    increments, including the opposite orientation. At equal orientation error,
    prefer the elbow branch nearest prev; otherwise use a deterministic order.
    Returned phi is the actual joint-angle sum, as in forward().
    """
    if not isfinite(step) or step <= 0.0:
        raise ValueError('step must be a positive finite angle')
    start = 180.0 if phi_hint is None else phi_hint
    start = (start + 180.0) % 360.0 - 180.0
    for k in range(ceil(180.0 / step) + 1):
        offset = min(k * step, 180.0)
        orientations = ((start + offset,) if offset in (0.0, 180.0)
                        else (start + offset, start - offset))
        valid = []
        for phi in orientations:
            valid.extend(q for q in _solutions(r, z, phi)
                         if within_limits(q) and clears_ground(q))
        if valid:
            q = (valid[0] if prev is None else
                 min(valid, key=lambda q: sum((a - b) ** 2
                                             for a, b in zip(q, prev))))
            return q, sum(q) - 270.0
    return None
