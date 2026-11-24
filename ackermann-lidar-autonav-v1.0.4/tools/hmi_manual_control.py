#!/usr/bin/env python3
"""Safety-oriented manual Ackermann commands, independent of Qt and ROS.

Default: dead-man press-and-hold driving; numeric latching requires an
explicit Apply action in the HMI. F is a motor-force command, NOT a brake.
"""
from __future__ import annotations

import math

DIRECTIONS = frozenset(('forward', 'reverse', 'left', 'right'))


def clamp(value, limit):
    limit = max(0.0, float(limit))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('Command must be finite')
    return max(-limit, min(limit, value))


class ManualDrive:
    def __init__(self, max_force_N, max_steer_deg):
        if max_force_N <= 0 or max_steer_deg <= 0:
            raise ValueError('Invalid physical limits')
        self.max_force_N = float(max_force_N)
        self.max_steer_deg = float(max_steer_deg)
        self.force_step_N = min(80.0, self.max_force_N)
        self.steer_step_deg = min(15.0, self.max_steer_deg)
        self.held = set()
        self.latched = None

    def set_sensitivity(self, force_N, steer_deg):
        self.force_step_N = max(0.0, clamp(force_N, self.max_force_N))
        self.steer_step_deg = max(0.0, clamp(steer_deg, self.max_steer_deg))

    def press(self, direction):
        if direction not in DIRECTIONS:
            raise ValueError('Unknown driving direction')
        self.latched = None
        self.held.add(direction)

    def release(self, direction):
        self.held.discard(direction)

    def latch_numeric(self, force_N, steer_deg):
        self.held.clear()
        self.latched = (clamp(force_N, self.max_force_N),
                        clamp(steer_deg, self.max_steer_deg))

    def zero(self):
        self.held.clear()
        self.latched = None

    def command(self):
        if self.latched is not None:
            return self.latched
        throttle = int('forward' in self.held) - int('reverse' in self.held)
        turn = int('left' in self.held) - int('right' in self.held)
        return throttle * self.force_step_N, turn * self.steer_step_deg
