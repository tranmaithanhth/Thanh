#!/usr/bin/env python3
"""Regression: F=15N while XY stays near (2.34,1.75) and odom v fluctuates."""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from traction_recovery_v104 import DisplacementWatchdog
from local_planners_v104 import LocalPlanningMission
from planner_core import Pose


class TestTractionRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((ROOT/'mission.json').read_text())

    def watchdog(self):
        c=self.cfg
        return DisplacementWatchdog(c['traction_window_s'],
                                    c['traction_min_xy_m'],
                                    c['traction_min_vref_m_s'])

    def test_stalled_xy_requests_bounded_force_despite_odom_speed(self):
        w=self.watchdog();forces=[]
        for k in range(85):
            t=k*.1
            x=2.34+.005*math.sin(t*3.)
            y=1.75+.006*math.cos(t*5.)
            forces.append(w.force_floor(t,x,y,.25,True,65.,22.,145.,.35))
        self.assertGreater(max(forces),65.)
        self.assertLessEqual(max(forces),145.)
        self.assertLess(w.xy_speed,.035)

    def test_translating_xy_is_not_stalled(self):
        w=self.watchdog()
        for k in range(90):
            t=k*.1
            self.assertEqual(
                w.force_floor(t,1.6+.24*t,.83+.10*t,.3,True,65.,22.,145.,.35),
                0.)

    def test_reset_for_brake_or_real_progress(self):
        w=self.watchdog()
        for k in range(50):
            w.force_floor(k*.1,2.34,1.75,.25,True,65,22,145,.35)
        self.assertGreater(w.force_floor(5.1,2.34,1.75,.25,True,65,22,145,.35),0)
        self.assertEqual(w.force_floor(5.2,2.34,1.75,0.,True,65,22,145,.35),0)
        self.assertIsNone(w.stalled_since)
        for k in range(50):
            w.force_floor(6.+k*.1,2.34,1.75,.25,True,65,22,145,.35)
        self.assertEqual(w.force_floor(11.1,2.65,1.85,.25,True,65,22,145,.35),0)

    def test_log_190619_low_dwa_samples_do_not_reset_real_xy_watch(self):
        # On the user's 19:06:08-19:06:19 trace, DWA oscillates between
        # vref=0.04 and 0.30m/s while P~(9.06,4.25) never advances.
        # Old code used selected DWA vref directly, resetting the sensor
        # every time a sample <0.15 arrives (XY_speed=nan, F_floor=0).
        w_old=self.watchdog();w_new=self.watchdog()
        chosen=[.30,.14,.16,.04,.23,.14,.05,.22,.04,.28]
        old_forces=[];new_forces=[]
        for k in range(120):
            t=.1*k
            x=9.06+.003*math.sin(t*2.)
            y=4.25+.004*math.cos(t*3.)
            old_forces.append(w_old.force_floor(
                t,x,y,chosen[k%len(chosen)],True,65.,22.,145.,.35))
            new_forces.append(w_new.force_floor(
                t,x,y,float(self.cfg['traction_intent_speed_m_s']),
                True,65.,22.,145.,.35))
        self.assertEqual(max(old_forces),0.0)
        self.assertGreater(max(new_forces),80.)
        self.assertTrue(math.isfinite(w_new.xy_speed))
        self.assertLess(w_new.xy_speed,.035)

    def test_reported_dwa_pose_recovery_enforces_valid_trajectory(self):
        # DWA movement remains bounded by known nearby objects, including
        # the fourth model discovered near (12.4,3.7).
        m=LocalPlanningMission(dict(self.cfg))
        m.update_dynamic([
            ('obstacle_02',7.3,1.15,.75),
            ('obstacle_03',10.15,.35,.75),
            ('obstacle_04',12.4,3.7,.75)])
        m.astar((7.66,3.72))
        pose=Pose(9.06,4.25,math.radians(25.1),.12)
        m.traction_force_floor_N=65.
        cmd=m.local_force_steer(pose)
        self.assertEqual(m.last_solver,'DWA')
        self.assertGreater(m.last_dwa_valid_candidates,0)
        self.assertGreaterEqual(cmd[4],.035)
        self.assertGreaterEqual(cmd[0],65.)
        self.assertLessEqual(cmd[0],m.max_F)
        self.assertLessEqual(abs(cmd[1]),m.max_delta)
        for x,y in m.local_path:
            self.assertTrue(m.safe_xy(x,y))

    def test_nav_uses_global_route_intent_not_last_dwa_vref(self):
        nav=(ROOT/'tools/autonomous_nearby_nav.py').read_text()
        self.assertIn('intent_vref=self.mission.drive_direction',nav)
        self.assertIn('now,pose.x,pose.y,intent_vref,enabled=eligible',nav)
        self.assertIn("self.mission.mode=='DWA'",nav)
        # Proven TEB remains untouched by the new DWA-only stall boost.
        scope=nav[nav.index('eligible=(not self.dry_run'):][:380]
        self.assertIn("self.mission.mode=='DWA'",scope)
        self.assertNotIn("mode in ('DWA','TEB')",scope)

    def test_dwa_predicts_boost_and_keeps_obstacle_guards(self):
        m=LocalPlanningMission(dict(self.cfg))
        m.update_dynamic([('obstacle_01',4.1,.3,.75)])
        m.astar((1.6,.83))
        p=Pose(2.34,1.75,math.radians(46.),.14)
        m.traction_force_floor_N=65.0
        self.assertGreaterEqual(m._force(p,.25,13.71),65.)
        candidate=m._dwa(p,m.feedback(p))
        self.assertIsNotNone(candidate)
        self.assertGreaterEqual(candidate[0],65.)
        self.assertGreater(m.last_dwa_valid_candidates,0)
        for x,y in candidate[5]:
            self.assertTrue(m.safe_xy(x,y))

    def test_zero_speed_never_applies_recovery_boost(self):
        m=LocalPlanningMission(dict(self.cfg))
        pose=Pose(2.,1.,0.,0.)
        base=m._force(pose,0.,13.7)
        # Even with vref=0, the existing controller uses gravitational
        # feedforward on a sloped surface. That ~4 N is not traction boost.
        m.traction_force_floor_N=140.
        self.assertAlmostEqual(m._force(pose,0.,13.7),base,places=7)
        self.assertLess(abs(base),140.)

    def test_live_code_checks_collision_before_traction_floor(self):
        nav=(ROOT/'tools/autonomous_nearby_nav.py').read_text()
        for anchor in ('self.traction_watch.force_floor(',
                       'self.mission.rollout_safe(',
                       'self.mission.traction_force_floor_N=floor',
                       '[XY TRACTION RECOVERY]', 'slip_possible'):
            self.assertIn(anchor,nav)


if __name__=='__main__':
    unittest.main()
