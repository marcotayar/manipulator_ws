#!/usr/bin/env python3
"""
2D inverse kinematics for the 3-link planar arm (shoulder, elbow, wrist).
Keeps the end-effector pointing down as much as possible to spare the
weak wrist servo (MG90S).

Geometry (y = vertical, x = horizontal, base yaw handled separately):
  Shoulder (joint A): fixed at (0, 0.09)
  Link A->B (shoulder->elbow): 0.10 m
  Link B->C (elbow->wrist):    0.09 m
  Link C->tip (wrist->EE tip): 0.16 m
  All joints: +/- 90 deg (pi/2)
  Constraint: tip y >= 0
"""
from math import atan2, sqrt, acos, pi, cos, sin, radians, log10

# Geometry
BASE_X = 0.0
BASE_Y = 0.09
L1 = 0.10   # shoulder -> elbow
L2 = 0.09   # elbow -> wrist
L3 = 0.16   # wrist -> tip

# SAFETY: the gripper tip must never go below this height. Below ground the
# gripper jams into the table and destroys the shoulder gear. Raise for more
# hover margin; never set below 0.
GROUND_CLEAR = 0.0

J_MIN = -pi / 2
J_MAX = pi / 2

# Per-joint limits kept clear of the servo mechanical locks. The firmware
# clamps every positional servo to 10-170°; these IK limits mirror that so the
# solver never asks for a pose that would slam a gear into its end stop.
#   shoulder servo = 0  + deg(ik)  -> ik in [10, 90]
#   elbow    servo = 90 - deg(ik)  -> ik in [-80, 80]
#   wrist    servo = 90 + deg(ik)  -> ik in [-80, 80]
SH_MIN, SH_MAX = radians(10), radians(90)
EL_MIN, EL_MAX = radians(-80), radians(80)
WR_MIN, WR_MAX = radians(-80), radians(80)

# Joint-limit comparison tolerance. The safe start pose sits exactly on
# SH_MAX and WR_MIN, and the FK->IK round trip lands a few ulps outside them.
# Without this slack the solver rejects its own start pose.
LIMIT_EPS = 1e-6

# Safe start pose — MUST match esp32_microros.ino setup(). Gripper hovers above
# ground, low shoulder load. Single source of truth for all nodes.
START_SHOULDER = radians(90)
START_ELBOW = radians(-60)
START_WRIST = radians(-80)

# /arm_command action vector. J1 is a continuous-rotation servo, so its action
# is velocity rather than an absolute yaw angle.
ACTION_SPACE = (
    ('base velocity', -1.0, 1.0, 'normalized'),
    ('shoulder', SH_MIN, SH_MAX, 'rad'),
    ('elbow', EL_MIN, EL_MAX, 'rad'),
    ('wrist', WR_MIN, WR_MAX, 'rad'),
    ('gripper', 0.0, 1.0, 'open..closed'),
)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _in_limits(t1, t2, t3):
    """Joint limits with LIMIT_EPS slack, so on-limit poses stay solvable."""
    return (SH_MIN - LIMIT_EPS <= t1 <= SH_MAX + LIMIT_EPS and
            EL_MIN - LIMIT_EPS <= t2 <= EL_MAX + LIMIT_EPS and
            WR_MIN - LIMIT_EPS <= t3 <= WR_MAX + LIMIT_EPS)


def cartesian_to_cylindrical(x, y, z):
    """Return ``(yaw, radius, height)`` for a world-frame Cartesian target."""
    return atan2(y, x), sqrt(x * x + y * y), z


def format_action_space():
    """Human-readable description of the five-element hardware command."""
    lines = []
    for index, (name, lower, upper, unit) in enumerate(ACTION_SPACE):
        lines.append(
            f'  [{index}] {name:<13} [{lower:+.3f}, {upper:+.3f}] {unit}')
    return '\n'.join(lines)


def _solve_2link(wx, wy, l1, l2, elbow_up):
    """2-link IK reaching wrist (wx, wy) from origin. Returns (t1, t2) or None."""
    d_sq = wx * wx + wy * wy
    d = sqrt(d_sq)
    if d > (l1 + l2) or d < abs(l1 - l2):
        return None
    cos_t2 = clamp((d_sq - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1.0, 1.0)
    t2 = acos(cos_t2)
    if elbow_up:
        t2 = -t2
    k1 = l1 + l2 * cos(t2)
    k2 = l2 * sin(t2)
    t1 = atan2(wy, wx) - atan2(k2, k1)
    return t1, t2


def solve_fixed_phi(tip_x, tip_y, phi):
    """Solve 3-link IK for a given EE approach angle phi (rad). Returns dict or None."""
    if tip_y < GROUND_CLEAR:
        return None

    wx = tip_x - L3 * cos(phi)
    wy = tip_y - L3 * sin(phi)
    wx_s = wx - BASE_X
    wy_s = wy - BASE_Y

    for elbow_up in (True, False):
        sol = _solve_2link(wx_s, wy_s, L1, L2, elbow_up)
        if sol is None:
            continue
        t1, t2 = sol
        t3 = phi - (t1 + t2)
        if not _in_limits(t1, t2, t3):
            continue
        return {
            'shoulder': clamp(t1, SH_MIN, SH_MAX),
            'elbow': clamp(t2, EL_MIN, EL_MAX),
            'wrist': clamp(t3, WR_MIN, WR_MAX),
            'phi': phi,
            'elbow_up': elbow_up,
        }
    return None


def solve_fixed_phi_all(tip_x, tip_y, phi):
    """All valid configs (both elbow branches) for a given EE angle phi."""
    out = []
    if tip_y < GROUND_CLEAR:
        return out

    wx = tip_x - L3 * cos(phi)
    wy = tip_y - L3 * sin(phi)
    wx_s = wx - BASE_X
    wy_s = wy - BASE_Y

    for elbow_up in (True, False):
        sol = _solve_2link(wx_s, wy_s, L1, L2, elbow_up)
        if sol is None:
            continue
        t1, t2 = sol
        t3 = phi - (t1 + t2)
        if not _in_limits(t1, t2, t3):
            continue
        out.append({
            'shoulder': clamp(t1, SH_MIN, SH_MAX),
            'elbow': clamp(t2, EL_MIN, EL_MAX),
            'wrist': clamp(t3, WR_MIN, WR_MAX),
            'phi': phi,
            'elbow_up': elbow_up,
        })
    return out


def solve(tip_x, tip_y, prev=None):
    """
    Down-preferred IK with continuity.

    Hard priority: EE points as close to straight down (phi = -90 deg) as
    the target allows. Among configs that are equally down (the two elbow
    branches, or the +/- phi pair at the same tilt), the one closest to
    `prev` is chosen so the arm doesn't jump/jitter between solutions.

    prev: optional dict with 'shoulder','elbow','wrist' (last commanded pose).
    Returns dict {shoulder, elbow, wrist, phi, elbow_up} or None.
    """
    if tip_y < GROUND_CLEAR:
        return None

    phis = [-pi / 2]
    step = radians(1)            # fine grid → smooth phi transitions, no snapping
    for i in range(1, 181):
        phis.append(-pi / 2 + i * step)
        phis.append(-pi / 2 - i * step)

    cands = []
    for phi in phis:
        cands.extend(solve_fixed_phi_all(tip_x, tip_y, phi))

    if not cands:
        return None

    # Primary: most "down" — smallest tilt away from straight down.
    best_dist = min(abs(c['phi'] + pi / 2) for c in cands)
    tol = radians(0.5)
    near = [c for c in cands if abs(c['phi'] + pi / 2) <= best_dist + tol]

    # Secondary: closest to the previous pose (kills jumps/jitter).
    if prev is None:
        return near[0]

    def pose_dist2(c):
        return ((c['shoulder'] - prev['shoulder']) ** 2 +
                (c['elbow'] - prev['elbow']) ** 2 +
                (c['wrist'] - prev['wrist']) ** 2)

    return min(near, key=pose_dist2)


def solve_cartesian(x, y, z, prev=None):
    """Solve a world target as cylindrical yaw plus the planar arm IK.

    The returned ``base_yaw`` is directly usable by the URDF simulation. On
    physical hardware it is a reference only: the continuous-rotation base
    servo has no angular feedback and accepts velocity, not position.
    """
    base_yaw, reach, height = cartesian_to_cylindrical(x, y, z)
    solution = solve(reach, height, prev)
    if solution is None:
        return None
    return {
        'base_yaw': base_yaw,
        'reach': reach,
        'height': height,
        **solution,
    }


def forward(shoulder, elbow, wrist):
    """FK: returns tip (x, y) for the three planar joint angles."""
    x = BASE_X + L1 * cos(shoulder)
    y = BASE_Y + L1 * sin(shoulder)
    x += L2 * cos(shoulder + elbow)
    y += L2 * sin(shoulder + elbow)
    x += L3 * cos(shoulder + elbow + wrist)
    y += L3 * sin(shoulder + elbow + wrist)
    return x, y


def nearest_feasible(reach, height, step=0.001, max_rings=40):
    """Snap a planar (reach, height) target to the nearest solvable grid point.

    The safe start pose sits on the workspace boundary, so a UI that displays
    millimetre precision can round it to an unsolvable point. This returns the
    closest point on the ``step`` grid that ``solve`` accepts, or ``None``.
    """
    digits = max(0, round(-log10(step)))
    base_r = round(reach, digits)
    base_h = round(height, digits)

    best = None
    best_d2 = float('inf')
    for ring in range(max_rings + 1):
        for i in range(-ring, ring + 1):
            for j in range(-ring, ring + 1):
                if max(abs(i), abs(j)) != ring:
                    continue          # only the new ring, inner ones are done
                r = round(base_r + i * step, digits)
                h = round(base_h + j * step, digits)
                if h < GROUND_CLEAR or solve(r, h) is None:
                    continue
                d2 = (r - reach) ** 2 + (h - height) ** 2
                if d2 < best_d2:
                    best_d2, best = d2, (r, h)
        if best is not None:
            return best
    return None
