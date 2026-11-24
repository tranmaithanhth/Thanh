import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'tools'))
from planner_core import Mission, Pose, terrain, smootherstep
from patch_robot_v103 import patch


class TestTerrainAndAutonomy(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((BASE/'mission.json').read_text())
        cls.m=Mission(cls.cfg)

    def test_terrain_like_matlab(self):
        for x in [-10,2.5,4.5,6.5,10.6,14.6,25]:
            h,hx,hy=terrain(x,-1.2)
            eps=1e-4
            numx=(terrain(x+eps,-1.2)[0]-terrain(x-eps,-1.2)[0])/(2*eps)
            numy=(terrain(x,-1.2+eps)[0]-terrain(x,-1.2-eps)[0])/(2*eps)
            self.assertAlmostEqual(hx,numx,places=4)
            self.assertAlmostEqual(hy,numy,places=4)

    def test_grade_threshold_allows_real_ramp(self):
        # At x≈4.5, y=-2 the MATLAB terrain gradient is slightly above 0.50.
        # Old 0.50 cutoff blocked the straight/lower corridor, forcing an
        # almost-tangent upper route around the obstacle.
        h,hx,hy=terrain(4.5,-2.1)
        self.assertGreater(math.hypot(hx,hy),0.50)
        self.assertLess(math.hypot(hx,hy),self.cfg['max_terrain_grade'])
        self.assertTrue(self.m.safe_xy(4.5,-2.1,margin=self.m.route_margin))

    def test_astar_leaves_route_clearance(self):
        route=self.m.astar((-0.4,-2.0))
        self.assertGreaterEqual(self.m.route_margin,0.30)
        for a,b in zip(route,route[1:]):
            length=math.dist(a,b)
            for k in range(max(2,math.ceil(length/0.05))+1):
                t=k/max(2,math.ceil(length/0.05))
                x=a[0]+t*(b[0]-a[0])
                y=a[1]+t*(b[1]-a[1])
                for ox,oy,r in self.m.obstacles:
                    self.assertGreater(math.hypot(x-ox,y-oy),
                                       r+self.m.infl+self.m.route_margin-0.000001)

    def test_forward_progress_not_negative_when_stopped(self):
        self.m.astar((-0.4,-2.0))
        p=Pose(1.8,-2.05,0.0,0.03)
        F,delta,dist,dev,vref=self.m.local_force_steer(p)
        self.assertGreater(vref,0.25)
        self.assertGreaterEqual(F,self.m.min_progress_force-1e-6)

    def test_rollout_keeps_speed_above_cruise_setpoint(self):
        p=Pose(0.0,-2.0,0.0,1.05)
        next_p=self.m.step(p,0.0,0.0,0.1)
        self.assertGreater(next_p.v,self.cfg['max_speed_m_s'])
        self.assertLessEqual(next_p.v,self.cfg['physical_max_speed_m_s'])

    def test_new_safe_start_inside_extra_route_margin(self):
        # The inflated physical radius is 1.95m; the additional route
        # buffer increases it to 2.30m. This is a valid *physical* start.
        mission=Mission(self.cfg)
        start=(4.1,-1.82)
        self.assertTrue(mission.safe_xy(*start,margin=0.0))
        self.assertFalse(mission.safe_xy(*start,margin=mission.route_margin))
        path=mission.astar(start)
        self.assertEqual(path[0],start)
        self.assertEqual(path[-1],(16.0,0.6))
        self.assertTrue(mission.segment_safe(path[0],path[1],margin=0.0))
        for a,b in zip(path[1:],path[2:]):
            self.assertTrue(mission.segment_safe(a,b))

    def test_refuses_physical_collision_start(self):
        mission=Mission(self.cfg)
        with self.assertRaises(ValueError):
            mission.astar((4.1,-1.5))

    def test_rotated_spawn_uses_reverse_instead_of_wrong_forward(self):
        mission=Mission(self.cfg)
        mission.astar((0.0,-2.0))
        p=Pose(0.0,-2.0,math.pi,0.0)
        f,delta,dist,dev,vref=mission.local_force_steer(p)
        self.assertLess(vref,-0.05)
        self.assertLess(f,0.0)
        self.assertLessEqual(abs(f),mission.max_F)
        self.assertLessEqual(abs(delta),mission.max_delta)

    def test_reverse_overspeed_penalty_is_signed(self):
        src=(BASE/'tools/planner_core.py').read_text()
        self.assertIn('(1 if vref>=0 else -1)*(pred.v-vref)',src)
        self.assertIn('self.drive_direction=-1 if err>math.radians(105) else 1',src)

    def test_astar_segments_clear(self):
        for start in [(-0.4,-2.0),(-4,-2),(0,0)]:
            route=self.m.astar(start)
            self.assertEqual(route[0],start)
            self.assertEqual(route[-1],(16.0,0.6))
            for a,b in zip(route,route[1:]):
                self.assertTrue(self.m.segment_safe(a,b),f'blocked segment {a}->{b}')

    def test_boosted_force_config_consistent(self):
        params=json.loads((BASE/'matlab_vehicle_params.json').read_text())
        self.assertAlmostEqual(params['matlab_reference_rear_axle_torque_Nm'], 22.)
        self.assertAlmostEqual(params['enhanced_max_total_drive_torque_Nm'],45.)
        self.assertEqual(params['drive_mode'],'all')
        self.assertAlmostEqual(params['max_drive_force_N'] * params['radius_m'],
                               params['enhanced_max_total_drive_torque_Nm'])
        self.assertAlmostEqual(self.cfg['max_drive_force_N'],params['max_drive_force_N'])
        self.assertLessEqual(self.cfg['hill_start_force_N'],self.cfg['max_drive_force_N'])

    def test_hill_start_boost_only_on_uphill(self):
        self.m.astar((-0.4,-2))
        # At x=4.5 the smooth ramp has a positive grade near its maximum.
        p=Pose(4.5,-2.5,0.0,0.0)
        # Test the underlying feedback force floor, not rollout choice or Gazebo traction.
        f,d,tgt,dist,dev,vref=self.m.feedback(p)
        self.assertGreaterEqual(f, self.cfg['hill_start_force_N'] - 1e-6)
        self.assertLessEqual(abs(f),self.m.max_F+1e-7)

    def test_front_axle_on_ramp_before_rear(self):
        # This reproduces the measured Gazebo stall:
        # rear P=(2.35,-1.75), yaw~4 deg, v~0.12, while the nose is uphill.
        self.m.astar((-0.4,-2))
        p=Pose(2.35,-1.75,math.radians(4.0),0.12)
        rear,front=self.m.axle_grades(p)
        self.assertLess(rear,0.025)
        self.assertGreater(front,0.12)
        self.assertGreater(front,self.cfg['hill_start_min_grade'])
        self.assertGreater(self.m.grade_for_dynamics(p),rear)
        f0,delta,_,dist,dev,vref=self.m.feedback(p)
        self.assertGreaterEqual(f0,self.cfg['hill_start_force_N']-1e-6)
        self.assertTrue(self.m.rollout_safe(p,175.0,delta,0.45))

    def test_mpc_does_not_cancel_hill_start_force(self):
        self.m.astar((-0.4,-2))
        p=Pose(2.35,-1.75,math.radians(4.0),0.12)
        F,delta,dist,dev,vref=self.m.local_force_steer(p)
        self.assertGreater(vref,0.25)
        self.assertGreaterEqual(F,self.cfg['hill_start_force_N']-1e-6)
        self.assertLessEqual(F,self.m.max_F)
        self.assertLessEqual(abs(delta),self.m.max_delta)

    def test_hill_start_force_tapers_smoothly(self):
        self.m.astar((-0.4,-2))
        yaw=math.radians(4.0)
        f_slow=self.m.hill_start_force_floor(Pose(2.35,-1.75,yaw,0.12),0.75,13.84)
        f_mid=self.m.hill_start_force_floor(Pose(2.35,-1.75,yaw,0.32),0.75,13.84)
        f_fast=self.m.hill_start_force_floor(Pose(2.35,-1.75,yaw,0.46),0.75,13.84)
        f_flat=self.m.hill_start_force_floor(Pose(0.0,-2.0,0.0,0.12),0.75,16.0)
        self.assertAlmostEqual(f_slow,self.cfg['hill_start_force_N'],places=6)
        self.assertGreater(f_mid,0.0)
        self.assertLess(f_mid,f_slow)
        self.assertEqual(f_fast,0.0)
        self.assertEqual(f_flat,0.0)

    def test_recovery_config_stays_within_torque_limit(self):
        self.assertGreater(self.cfg['recovery_after_s'],0.0)
        self.assertGreater(self.cfg['recovery_initial_force_N'],0.0)
        self.assertLessEqual(self.cfg['recovery_max_force_N'],self.m.max_F)
        self.assertGreater(self.cfg['recovery_ramp_N_per_s'],0.0)
        self.assertGreater(self.cfg['recovery_horizon_s'],0.0)

    def test_bounded_commands(self):
        self.m.astar((-0.4,-2))
        for state in [Pose(-.4,-2,0,0),Pose(4.,-1.,.2,.4),Pose(16,.6,0,.2)]:
            # Last state can be within obstacle? (16,.6) is clear
            F,delta,dist,dev,vref=self.m.local_force_steer(state)
            self.assertLessEqual(abs(F),self.m.max_F+1e-7)
            self.assertLessEqual(abs(delta),self.m.max_delta+1e-7)

    def test_closed_loop_reaches_goal_in_reduced_model(self):
        p=Pose(-.4,-2,0,0)
        self.m.astar((p.x,p.y))
        for i in range(1200):
            F,d,dist,dev,vref=self.m.local_force_steer(p)
            self.assertTrue(self.m.safe_motion(p.x,p.y,p.yaw), (i,p))
            if dist<self.cfg['goal_tolerance_m'] and abs(p.v)<0.06:
                break
            p=self.m.step(p,F,d,.1)
        else:
            self.fail(f'Reduced simulated tracker failed to reach goal: {p}')
        self.assertLess(i/10,120)


class TestRelocationAndSpawn(unittest.TestCase):
    def test_teleport_replans(self):
        nav=(BASE/'tools/autonomous_force_nav.py').read_text(encoding='utf-8')
        self.assertIn('self.pending_replan=True',nav)
        self.assertIn('if self.pending_replan:',nav)
        self.assertIn('self.mission.path=[]',nav)
        self.assertIn('self.mission.drive_direction*=-1',nav)

    def test_spawn_initial_height_and_yaw_options(self):
        script=(BASE/'spawn_force_v103.sh').read_text(encoding='utf-8')
        self.assertIn('ACKERMANN_SPAWN_YAW',script)
        self.assertIn('ACKERMANN_SPAWN_Z:-auto',script)
        self.assertIn('terrain(x,y)[0]',script)
        self.assertIn('-Y "$SPAWN_YAW"',script)


class TestHillRecoveryGuard(unittest.TestCase):
    def test_recovery_only_for_stagnant_uphill_with_safe_rollout(self):
        nav=(BASE/'tools/autonomous_force_nav.py').read_text(encoding='utf-8')
        self.assertIn("if not self.dry_run and self.last_progress is not None and vref>0.25:",nav)
        self.assertIn("max(rear_grade,front_grade)>=",nav)
        self.assertIn("stagnant_s>=",nav)
        self.assertIn("self.mission.rollout_safe(",nav)
        self.assertIn("min(\n                    self.mission.max_F",nav)

class TestDryRunProgressGuard(unittest.TestCase):
    def test_dry_run_skips_progress_watchdog(self):
        # Static source guard because the ROS 2 node cannot be instantiated
        # in this no-ROS, offline planner test suite.
        nav=(BASE/'tools/autonomous_force_nav.py').read_text(encoding='utf-8')
        self.assertIn('if not self.dry_run:',nav)
        self.assertIn('stationary odometry is an expected result',nav)
        self.assertIn('self.last_progress=(now,pose.x,pose.y)',nav)
        self.assertIn('NO force_steer commands are published',nav)


class TestForcePluginScheduling(unittest.TestCase):
    def test_force_set_on_every_physics_step(self):
        # Regression: Gazebo world uses 1ms step. SetForce must not be
        # throttled to the 10ms (or larger) telemetry update interval.
        src=(BASE/'ackermann_force_control/src/ackermann_force_drive.cpp').read_text()
        self.assertIn('if (dt <= 0.0) return;',src)
        self.assertNotIn('dt < 0.01',src)
        self.assertIn('last_publish_',src)
        gate=src.index('if ((info.simTime - last_publish_).Double() < 0.02) return;')
        for command in ('rear_l_->SetForce', 'rear_r_->SetForce',
                        'front_l_->SetForce', 'front_r_->SetForce',
                        'pd(steer_l_', 'pd(steer_r_'):
            self.assertLess(src.index(command),gate,command+' must execute every tick')

class TestUrdfPhysicalAlignment(unittest.TestCase):
    def test_aggregate_mass_and_force_plugin(self):
        params=json.loads((BASE/'matlab_vehicle_params.json').read_text())
        root=ET.Element('robot',{'name':'ackermann_vehicle'})
        for name in ['chassis','rear_left_wheel','rear_right_wheel',
                     'front_left_wheel','front_right_wheel',
                     'front_left_steering_link','front_right_steering_link']:
            link=ET.SubElement(root,'link',{'name':name})
            inert=ET.SubElement(link,'inertial')
            ET.SubElement(inert,'mass',{'value':'1.0'})
            ET.SubElement(inert,'inertia',{'ixx':'0.1','iyy':'0.1','izz':'0.1'})
            if 'wheel' in name:
                gz=ET.SubElement(root,'gazebo',{'reference':name})
                ET.SubElement(gz,'mu1').text='1000';ET.SubElement(gz,'mu2').text='1000'
                ET.SubElement(gz,'fdir1').text='0 0 1'
        gz=ET.SubElement(root,'gazebo')
        ET.SubElement(gz,'plugin',{'name':'ackermann_drive','filename':'libgazebo_ros_ackermann_drive.so'})
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'orig.urdf';dst=Path(d)/'patched.urdf'
            ET.ElementTree(root).write(src)
            mass=patch(src,dst,params)
            self.assertAlmostEqual(mass,20.0)
            out=ET.parse(dst).getroot()
            self.assertEqual(len(out.findall('.//plugin[@name="ackermann_force_drive"]')),1)
            self.assertEqual(len(out.findall('.//plugin[@name="ackermann_drive"]')),0)
            self.assertEqual(len(out.findall('.//fdir1')),0)
            for gz in out.findall('gazebo'):
                if 'wheel' in (gz.get('reference') or ''):
                    self.assertEqual(gz.find('mu1').text,'1.1')
                    self.assertEqual(gz.find('mu2').text,'1.1')
            plugin=out.find('.//plugin[@name="ackermann_force_drive"]')
            self.assertEqual(plugin.find('max_force').text,str(params['max_drive_force_N']))
            self.assertEqual(plugin.find('drive_mode').text,'all')


if __name__ == '__main__':unittest.main()