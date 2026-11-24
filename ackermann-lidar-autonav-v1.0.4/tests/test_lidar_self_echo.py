#!/usr/bin/env python3
"""Regression for orange LiDAR dot beside rear axle at X≈2.8,Y≈-2.0.

No ROS/Gazebo dependencies. Validate self-echo masking BEFORE both
dynamic occupancy and emergency braking are updated.
"""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from lidar_obstacles import (
    LidarObstacleLayer, is_self_return, first_terrain_intersection,
)


class TestLidarSelfEcho(unittest.TestCase):
    def setUp(self):
        self.config=json.loads((ROOT/'mission.json').read_text(encoding='utf-8'))
        self.layer=LidarObstacleLayer(self.config)

    def scan(self, distance, angle, yaw=0.0):
        # Model pose near the second screenshot, rear axle ~X=2.6m.
        cx,cy,cz=3.0,-2.0,0.12
        q=(0.0,0.0,math.sin(yaw/2),math.cos(yaw/2))
        rx=cx-0.4*math.cos(yaw)
        ry=cy-0.4*math.sin(yaw)
        return self.layer.update(
            [distance],angle,0.1,0.15,8.0,rx,ry,yaw,
            now=100.0,model_pose=(cx,cy,cz,*q))

    def test_self_echo_at_rear_of_body_does_not_brake_or_replan(self):
        # Scanner at world X=3.2, rear-facing echo at X=2.9:
        # above local flat ground yet INSIDE the vehicle footprint.
        self.assertTrue(self.scan(0.3,math.pi))
        self.assertEqual(self.layer.world_points(now=100),[])
        self.assertTrue(math.isinf(self.layer.nearest(-1,now=100)))
        self.assertEqual(self.layer.last_filter_stats['self'],1)

    def test_rotated_vehicle_self_echo_uses_chassis_heading(self):
        self.assertTrue(self.scan(0.3,math.pi,yaw=math.pi/2))
        self.assertEqual(self.layer.world_points(now=100),[])
        self.assertTrue(math.isinf(self.layer.nearest(-1,now=100)))
        self.assertEqual(self.layer.last_filter_stats['self'],1)

    def test_real_obstacle_outside_rear_bumper_still_brakes(self):
        # Range 1.0 behind scanner = X=2.2, 0.8m behind model origin.
        self.assertTrue(self.scan(1.0,math.pi))
        self.assertEqual(len(self.layer.world_points(now=100)),1)
        self.assertAlmostEqual(self.layer.nearest(-1,now=100),1.0)
        self.assertEqual(self.layer.last_filter_stats['self'],0)

    def test_robot_footprint_gate_is_not_planner_inflation(self):
        self.assertTrue(is_self_return(2.9,-2.0,3.0,-2.0,0.0))
        self.assertFalse(is_self_return(3.85,-2.0,3.0,-2.0,0.0))
        self.assertFalse(is_self_return(3.0,-2.6,3.0,-2.0,0.0))
        self.assertTrue(is_self_return(3.0,2.9,3.0,3.0,math.pi/2))

    def test_raycast_intersects_known_ramp_not_flat_ground(self):
        d=first_terrain_intersection(2.8,-2.0,0.4,1.0,0.0,0.0,4.0,
                                     self.config['height_shift_gazebo_m'])
        self.assertTrue(math.isfinite(d))
        self.assertGreater(d,0.6)
        self.assertLess(d,3.0)
        flat=first_terrain_intersection(0.0,-2.0,0.4,1.,0.,0.,1.0,
                                        self.config['height_shift_gazebo_m'])
        self.assertTrue(math.isinf(flat))


if __name__=='__main__':
    unittest.main()
