#!/usr/bin/env python3
"""Regression tests for the false vertical ramp wall seen near X≈5 m.

Works without ROS 2 / Gazebo; use real terrain equation and the same scan
projection as the v1.0.4 autonomous controller and HMI.
"""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from lidar_obstacles import LidarObstacleLayer, is_terrain_return
from planner_core import terrain


class TestGroundClassification(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads((ROOT/'mission.json').read_text(encoding='utf-8'))

    def new_layer(self):
        return LidarObstacleLayer(self.config)

    def scan_ray(self,layer,distance,sim_time=100.0):
        # Chassis at X=0, Y=-2, z=.05, LiDAR at x=.2,z=.38.
        return layer.update(
            [distance],0.0,0.1,0.15,8.0,
            -0.4,-2.0,0.0,
            now=sim_time,
            model_pose=(0.0,-2.0,0.05,0.0,0.0,0.0,1.0))

    def test_below_ramp_points_not_misclassified_as_dynamic_wall(self):
        # Raw 2D LiDAR hit at x=4.9, common on the ascending ramp.
        layer=self.new_layer()
        ramp_z=terrain(4.9,-2.0)[0]+self.config['height_shift_gazebo_m']
        scan_z=0.05+self.config['lidar_mount_z_m']
        self.assertGreater(ramp_z-scan_z,0.20)
        self.assertTrue(self.scan_ray(layer,4.7))
        self.assertEqual(layer.world_points(now=100.0),[])
        self.assertTrue(math.isinf(layer.nearest(1,now=100.0)))
        self.assertEqual(layer.last_filter_stats['terrain'],1)
        self.assertEqual(layer.last_filter_stats['below_map'],1)
        self.assertEqual(layer.last_filter_stats['dynamic'],0)

    def test_full_vertical_ramp_contour_is_filtered(self):
        # Reproduce the orange column at X≈4.9 across -5<Y<4.
        # Each independent 2D scan beam has a different bearing and range.
        for target_y in (-5.0,-3.0,-2.0,0.0,2.0,4.0):
            layer=self.new_layer()
            dx=4.9-0.2
            dy=target_y+2.0
            d=math.hypot(dx,dy)
            a=math.atan2(dy,dx)
            self.assertLess(d,8.0)
            ok=layer.update(
                [d],a,0.1,0.15,8.0,
                -0.4,-2.0,0.0,
                now=100.0,
                model_pose=(0.0,-2.0,0.05,0.0,0.0,0.0,1.0))
            self.assertTrue(ok)
            self.assertEqual(layer.world_points(now=100.0),[],
                             msg='false ramp return at Y='+str(target_y))

    def test_forward_obstacle_above_terrain_remains_visible(self):
        layer=self.new_layer()
        # Beam returns 1.8 m to x=2.0 while the road is still low.
        self.assertTrue(self.scan_ray(layer,1.8))
        self.assertEqual(len(layer.world_points(now=100.0)),1)
        self.assertAlmostEqual(layer.nearest(1,now=100.0),1.8)
        self.assertEqual(layer.last_filter_stats['terrain'],0)
        self.assertEqual(layer.last_filter_stats['dynamic'],1)

    def test_ground_height_margin_conservative(self):
        self.assertTrue(is_terrain_return(0.4,0.75,0.12))
        self.assertTrue(is_terrain_return(0.81,0.75,0.12))
        self.assertFalse(is_terrain_return(0.95,0.75,0.12))
        self.assertFalse(is_terrain_return(float('nan'),0.75,0.12))

    def test_expired_scan_is_not_reused(self):
        layer=self.new_layer()
        self.scan_ray(layer,1.8)
        self.assertEqual(len(layer.world_points(now=100.0)),1)
        self.assertEqual(layer.world_points(now=102.0),[])
        self.assertTrue(math.isnan(layer.nearest(1,now=102.0)))


if __name__=='__main__':
    unittest.main()
