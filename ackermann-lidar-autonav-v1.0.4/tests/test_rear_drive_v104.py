#!/usr/bin/env python3
"""RWD v1.0.4 configuration, URDF and C++ force routing regression tests.

The Gazebo physics engine itself is not available during these offline tests.
Physical tests must confirm 0 front motor torque in /force_state.
"""
from __future__ import annotations
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from patch_robot_v104 import patch


class TestRearDriveV104(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params=json.loads((ROOT/'matlab_vehicle_params.json').read_text())
        cls.mission=json.loads((ROOT/'mission.json').read_text())

    def test_v104_rear_drive_params_and_original_force_cap(self):
        self.assertEqual(self.params['drive_mode'],'rear')
        self.assertEqual(self.mission['drive_mode'],'rear')
        self.assertEqual(self.params['max_drive_force_N'],300)
        self.assertEqual(self.mission['max_drive_force_N'],300)
        self.assertAlmostEqual(self.params['radius_m'],0.15)
        self.assertAlmostEqual(self.params['enhanced_max_total_drive_torque_Nm'],45)
        self.assertAlmostEqual(
            self.params['max_drive_force_N']*self.params['radius_m']/2,22.5)
        self.assertAlmostEqual(float(self.params['total_mass_kg']),20.)

    def test_historical_v103_params_not_modified(self):
        old=ROOT.parent/'ackermann-force-trajectory-v1.0.3/matlab_vehicle_params.json'
        self.assertTrue(old.is_file())
        cfg=json.loads(old.read_text())
        self.assertEqual(cfg['drive_mode'],'all',
            'v1.0.3 AWD experiment must remain reproducible')

    def test_generated_urdf_has_four_wheels_but_only_rear_drivetrain(self):
        urdf=ET.Element('robot',{'name':'ackermann_vehicle'})
        names=('chassis','rear_left_wheel','rear_right_wheel',
               'front_left_wheel','front_right_wheel',
               'front_left_steering_link','front_right_steering_link')
        for name in names:
            link=ET.SubElement(urdf,'link',{'name':name})
            inert=ET.SubElement(link,'inertial')
            ET.SubElement(inert,'mass',{'value':'1'})
            ET.SubElement(inert,'inertia',{'ixx':'0.1','iyy':'0.1','izz':'0.1'})
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'stage3.urdf'
            dst=Path(tmp)/'rwd.urdf'
            ET.ElementTree(urdf).write(src)
            total_mass=patch(src,dst,self.params,use_lidar=False)
            root=ET.parse(dst).getroot()
            plugin=root.find('.//plugin[@name="ackermann_force_drive"]')
            self.assertIsNotNone(plugin)
            self.assertEqual(plugin.findtext('drive_mode'),'rear')
            self.assertEqual(plugin.findtext('max_force'),'300')
            self.assertAlmostEqual(total_mass,20.0)
            self.assertIsNone(root.find("link[@name='lidar_link']"))
            for name in names:
                self.assertIsNotNone(root.find(f"link[@name='{name}']"))

    def test_plugin_applies_torque_only_to_rear_and_steers_two_front(self):
        cpp=(ROOT.parent/'ackermann-force-trajectory-v1.0.3'/
             'ackermann_force_control/src/ackermann_force_drive.cpp').read_text()
        self.assertIn('drive_mode_ == "all" ? 4.0 : 2.0',cpp)
        self.assertIn('drive_mode_ == "all" ? total_torque / 4.0 : 0.0',cpp)
        self.assertIn('rear_l_->SetForce(0, each_rear_torque)',cpp)
        self.assertIn('rear_r_->SetForce(0, each_rear_torque)',cpp)
        self.assertIn('front_l_->SetForce(0, each_front_torque)',cpp)
        self.assertIn('front_r_->SetForce(0, each_front_torque)',cpp)
        self.assertIn('pd(steer_l_, delta_l)',cpp)
        self.assertIn('pd(steer_r_, delta_r)',cpp)
        # state.data[8]=per-rear torque, [9]=per-front motor torque;
        # front wheels may physically rotate by ground contact.
        self.assertIn('each_rear_torque, each_front_torque',cpp)

    def test_spawn_checks_shared_force_and_rear_drive_config(self):
        sh=(ROOT/'spawn_force_v104.sh').read_text()
        self.assertIn('PARAMS="$ROOT/matlab_vehicle_params.json"',sh)
        self.assertIn('--params "$V104_ROOT/matlab_vehicle_params.json"',sh)
        self.assertIn("params.get('drive_mode')!='rear'",sh)
        self.assertIn("mission.get('drive_mode')!='rear'",sh)
        self.assertIn("float(mission['max_drive_force_N'])",sh)
        p=subprocess.run(['bash','-n',str(ROOT/'spawn_force_v104.sh')],
                          capture_output=True,text=True)
        self.assertEqual(p.returncode,0,p.stderr)

    def test_runtime_checker_is_read_only(self):
        check=(ROOT/'tools/check_rwd_v104.py').read_text()
        self.assertIn("/ackermann_vehicle/force_state",check)
        self.assertIn('tau_front',check)
        self.assertIn('expected_each=f*0.15/2.',check)
        self.assertNotIn('create_publisher(',check)


if __name__=='__main__':
    unittest.main()
