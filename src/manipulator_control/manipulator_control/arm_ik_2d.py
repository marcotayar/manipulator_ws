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

J_MIN = -pi / 2
J_MAX = pi / 2


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
    if tip_y < 0:
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
        if not (J_MIN <= t1 <= J_MAX):
            continue
        if not (J_MIN <= t2 <= J_MAX):
            continue
        if not (J_MIN <= t3 <= J_MAX):
            continue
        return {
            'shoulder': t1,
            'elbow': t2,
            'wrist': t3,
            'phi': phi,
            'elbow_up': elbow_up,
        }
    return None


def solve(tip_x, tip_y):
    """
    Down-preferred IK. Tries phi = -90 deg (straight down) first, then
    sweeps outward, returning the reachable solution closest to vertical.
    Returns dict {shoulder, elbow, wrist, phi, elbow_up} or None.
    """
    candidates = [-pi / 2]
    step = radians(5)
    for i in range(1, 37):
        candidates.append(-pi / 2 + i * step)
        candidates.append(-pi / 2 - i * step)

    for phi in candidates:
        sol = solve_fixed_phi(tip_x, tip_y, phi)
        if sol is not None:
            return sol
    return None


def forward(shoulder, elbow, wrist):
    """FK: returns tip (x, y) for the three planar joint angles."""
    x = BASE_X + L1 * cos(shoulder)
    y = BASE_Y + L1 * sin(shoulder)
    x += L2 * cos(shoulder + elbow)
    y += L2 * sin(shoulder + elbow)
    x += L3 * cos(shoulder + elbow + wrist)
    y += L3 * sin(shoulder + elbow + wrist)
    return x, y
