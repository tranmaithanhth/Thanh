#!/usr/bin/env python3
"""ROS-free regression tests for Ackermann MATLAB-style DWA + reduced TEB."""
from __future__ import annotations

import json
import math
import sys
import time
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from local_planners_v104 import LocalPlanningMission,nelder_mead,nearest_s_point
from planner_core import Pose


class TestLocalPlanners(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads((ROOT/'mission.json').read_text())

    def mission(self,mode='DWA',start=(0.,-2.)):
        cfg=dict(self.config)
        cfg['local_planner']=mode
        m=LocalPlanningMission(cfg)
        m.astar(start)
        return m

    def assert_drive_feasible(self,m,cmd):
        F,steer,dist,dev,vref=cmd
        self.assertTrue(all(math.isfinite(float(x)) for x in cmd))
        self.assertLessEqual(abs(F),m.max_F+1e-5)
        self.assertLessEqual(abs(steer),m.max_delta+1e-5)
        self.assertLessEqual(abs(vref),m.max_speed+1e-5)

    def test_default_is_dwa_and_a_star_is_still_global(self):
        m=self.mission()
        self.assertEqual(m.mode,'DWA')
        self.assertEqual(m.obstacles,[])
        self.assertEqual(m.dynamic_obstacles,[])
        self.assertEqual(m.path[-1],tuple(self.config['goal_xy']))
        self.assertEqual(m.local_path,[])

    def test_dwa_produces_seven_point_collision_free_local_band(self):
        m=self.mission()
        pose=Pose(0.,-2.,0.,0.)
        out=m.local_force_steer(pose)
        self.assert_drive_feasible(m,out)
        self.assertEqual(len(m.local_path),7)
        self.assertAlmostEqual(m.local_path[0][0],pose.x)
        self.assertAlmostEqual(m.local_path[0][1],pose.y)
        for x,y in m.local_path:
            self.assertTrue(m.safe_xy(x,y))
        self.assertGreaterEqual(out[4],0.)
        self.assertLessEqual(abs(out[1]),m.c['max_steer_rate_rad_s']/
                             m.c['planner_control_hz']+1e-5)

    def test_speed_steer_dynamic_window(self):
        m=self.mission()
        m.last_delta=.2
        pose=Pose(.0,-2.0,0.0,.30)
        cmd=m.local_force_steer(pose)
        self.assert_drive_feasible(m,cmd)
        self.assertLessEqual(abs(cmd[1]-.2),
            m.c['max_steer_rate_rad_s']/m.c['planner_control_hz']+1e-5)

    def test_teb_seven_nodes_positive_timing_and_no_excess_controls(self):
        m=self.mission('TEB')
        pose=Pose(0.,-2.,0.,.20)
        self.assertEqual(m.mode,'TEB')
        m.local_force_steer(pose)
        # TEB may correctly fall back to DWA if the nonholonomic geometry
        # is not feasible; both publish valid 7-point local trajectories.
        self.assertEqual(len(m.local_path),7)
        self.assert_drive_feasible(m,m.local_force_steer(pose))
        band=m._reference_band(pose,7,3.6)
        end=band[-1]
        z=[v for pt in band[1:-1] for v in pt]+[math.log(.4)]*6
        pts,dt=m._unpack_band(z,pose,end,7)
        self.assertEqual(len(pts),7)
        self.assertEqual(len(dt),6)
        self.assertTrue(all(t>0 for t in dt))
        self.assertTrue(math.isfinite(m._teb_cost(
            z,pose,end,7,(.5,.0))))

    def test_teb_actually_optimizes_when_straight_route_is_feasible(self):
        m=self.mission('TEB')
        pose=Pose(0.,-2.,math.atan2(2.6,16.),.20)
        guide=m.feedback(pose)
        result=m._teb(pose,guide)
        self.assertIsNotNone(result,'TEB should not always fall back to DWA')
        self.assertEqual(len(result[5]),7)
        self.assertTrue(math.isfinite(result[-1]))
        self.assertTrue(m._teb_is_safe(pose,result[5]))

    def test_nearby_object_injected_by_detector_not_global_config(self):
        m=self.mission('DWA')
        route=list(m.path)
        m.update_dynamic([('unknown_from_gazebo',4.0,-2.0,.65)])
        self.assertEqual(m.obstacles,[])
        self.assertEqual(len(m.dynamic_obstacles),1)
        self.assertEqual(m.path,route)
        self.assertTrue(m.path_blocked((0.,-2.)))
        newroute=m.astar((0.,-2.))
        self.assertEqual(newroute[-1],tuple(self.config['goal_xy']))
        for x,y in newroute:
            self.assertTrue(m.safe_xy(x,y))

    def test_safety_fallback_stops_for_collision_at_start(self):
        m=self.mission('DWA')
        m.update_dynamic([('just_spawned',.2,-2.,.5)])
        cmd=m.local_force_steer(Pose(0.,-2.,0.,0.))
        self.assert_drive_feasible(m,cmd)
        # No safe forward rollout; legacy collision guard must also fail.
        self.assertEqual(cmd[4],0.0)
        self.assertTrue(m.fallback_active)

    def test_original_rollout_mode_remains_available_for_comparison(self):
        m=self.mission('ROLLOUT')
        self.assertEqual(len(m.local_path),0)
        cmd=m.local_force_steer(Pose(0.,-2.,0.,0.))
        self.assert_drive_feasible(m,cmd)
        self.assertEqual(m.local_path,[])

    def test_nelder_mead_improves_smooth_quadratic(self):
        f=lambda v:(v[0]-1.0)**2+2.0*(v[1]+2.0)**2
        _,initial,evals=nelder_mead(f,[0.,0.],[.2,.2],90)
        self.assertLess(initial,f([0.,0.]))
        self.assertLessEqual(evals,90)

    def test_runtime_launcher_hmi_selects_modes_and_visualizes_path(self):
        nav=(ROOT/'tools/autonomous_nearby_nav.py').read_text()
        hmi=(ROOT/'tools/hmi_v104.py').read_text()
        self.assertIn("self.mission=LocalPlanningMission(cfg)",nav)
        self.assertIn("--local-planner",nav)
        self.assertIn("LOCAL_PATH='/ackermann_vehicle/local_path'",nav)
        self.assertIn("self.publish_local_path()",nav)
        self.assertIn("self.local_planner_combo.addItem",hmi)
        self.assertIn("'--local-planner',planner",hmi)
        self.assertIn("label='Quỹ đạo cục bộ DWA/TEB'",hmi)
        self.assertNotIn("LaserScan",nav)


if __name__=='__main__':
    unittest.main()
