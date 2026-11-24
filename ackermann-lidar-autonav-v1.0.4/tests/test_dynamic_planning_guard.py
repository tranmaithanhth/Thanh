#!/usr/bin/env python3
"""ROS-free regression checks: LiDAR occupancy and v1.0.4 safety ordering."""
import json
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from lidar_planner import DynamicMission


class TestDynamicPlanningGuard(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'mission.json').read_text(encoding='utf-8'))
        self.mission=DynamicMission(self.cfg)

    def test_new_dynamic_obstacle_inflates_occupied_area(self):
        self.mission.update_dynamic([(0.,0.)])
        self.assertFalse(self.mission.safe_xy(0.,0.))
        self.mission.update_dynamic([])
        self.assertTrue(self.mission.safe_xy(0.,0.))

    def test_replan_is_requested_for_new_obstacle_ahead(self):
        self.mission.path=[(-1.,-2.),(3.,-2.)]
        self.mission.update_dynamic([(1.,-2.)])
        self.assertTrue(self.mission.path_blocked((-1.,-2.)))
        self.mission.update_dynamic([(1.,3.)])
        self.assertFalse(self.mission.path_blocked((-1.,-2.)))

    def test_close_obstacle_replan_precedes_final_hold(self):
        source=(ROOT/'tools/autonomous_lidar_nav.py').read_text(encoding='utf-8')
        # Stationary robot must try replan even inside the braking distance.
        pos_block=source.index('if self.mission.path and self.mission.path_blocked')
        pos_hold=source.index('if hazard_close:')
        self.assertLess(pos_block,pos_hold)
        self.assertIn('self.publish_route()',source)
        self.assertIn('stale odometry / Gazebo paused or frozen',source)
        self.assertIn('multiple force_steer publishers',source)


if __name__=='__main__':
    unittest.main()
