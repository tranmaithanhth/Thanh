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

    def test_astar_segments_clear(self):
        for start in [(-0.4,-2.0),(-4,-2),(0,0)]:
            route=self.m.astar(start)
            self.assertEqual(route[0],start)
            self.assertEqual(route[-1],(16.0,0.6))
            for a,b in zip(route,route[1:]):
                self.assertTrue(self.m.segment_safe(a,b),f'blocked segment {a}->{b}')

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
            self.assertEqual(out.find('.//plugin[@name="ackermann_force_drive"]/max_force').text,str(params['max_drive_force_N']))


if __name__ == '__main__':unittest.main()