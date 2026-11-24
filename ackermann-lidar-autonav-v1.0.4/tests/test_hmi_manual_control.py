#!/usr/bin/env python3
"""Offline checks for manual HMI safety (no Qt / ROS 2 required)."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from hmi_manual_control import ManualDrive


class TestManualDrive(unittest.TestCase):
    def setUp(self):
        self.pad = ManualDrive(300.0, 29.8)

    def test_no_command_without_pressed_deadman(self):
        self.assertEqual(self.pad.command(), (0.0, 0.0))
        self.pad.press('forward')
        self.assertEqual(self.pad.command(), (80.0, 0.0))
        self.pad.release('forward')
        self.assertEqual(self.pad.command(), (0.0, 0.0))

    def test_combined_direction_and_steering(self):
        self.pad.press('forward')
        self.pad.press('left')
        self.assertEqual(self.pad.command(), (80.0, 15.0))
        self.pad.release('left')
        self.assertEqual(self.pad.command(), (80.0, 0.0))
        self.pad.press('right')
        self.assertEqual(self.pad.command(), (80.0, -15.0))

    def test_conflicting_keys_resolve_to_zero(self):
        self.pad.press('forward')
        self.pad.press('reverse')
        self.pad.press('left')
        self.pad.press('right')
        self.assertEqual(self.pad.command(), (0.0, 0.0))

    def test_sensitivity_and_numeric_clamping(self):
        self.pad.set_sensitivity(120, 20)
        self.pad.press('reverse')
        self.assertEqual(self.pad.command(), (-120.0, 0.0))
        self.pad.latch_numeric(900, -75)
        self.assertEqual(self.pad.command(), (300.0, -29.8))
        self.pad.zero()
        self.assertEqual(self.pad.command(), (0.0, 0.0))

    def test_deadman_cancels_numeric_latch(self):
        self.pad.latch_numeric(55, 11)
        self.pad.press('left')
        self.assertEqual(self.pad.command(), (0.0, 15.0))
        self.pad.release('left')
        self.assertEqual(self.pad.command(), (0.0, 0.0))

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):
            self.pad.press('turnaround')
        with self.assertRaises(ValueError):
            self.pad.latch_numeric(float('nan'), 0)


if __name__ == '__main__':
    unittest.main()
