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
from math import atan2, sqrt, acos, pi, cos, sin, radians

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


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


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
        if not (SH_MIN <= t1 <= SH_MAX):
            continue
        if not (EL_MIN <= t2 <= EL_MAX):
            continue
        if not (WR_MIN <= t3 <= WR_MAX):
            continue
        return {
            'shoulder': t1,
            'elbow': t2,
            'wrist': t3,
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
        if not (SH_MIN <= t1 <= SH_MAX):
            continue
        if not (EL_MIN <= t2 <= EL_MAX):
            continue
        if not (WR_MIN <= t3 <= WR_MAX):
            continue
        out.append({
            'shoulder': t1,
            'elbow': t2,
            'wrist': t3,
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


def forward(shoulder, elbow, wrist):
    """FK: returns tip (x, y) for the three planar joint angles."""
    x = BASE_X + L1 * cos(shoulder)
    y = BASE_Y + L1 * sin(shoulder)
    x += L2 * cos(shoulder + elbow)
    y += L2 * sin(shoulder + elbow)
    x += L3 * cos(shoulder + elbow + wrist)
    y += L3 * sin(shoulder + elbow + wrist)
    return x, y
