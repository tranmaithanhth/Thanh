#!/usr/bin/env python3
"""Offline HMI tests; no PyQt5, ROS or Gazebo required in CI."""
from __future__ import annotations
import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'tools'))
from hmi_data import (
    FIELDS,COMPARED,numeric,quaternion_rpy,force_state_values,
    export_csv,load_matlab_csv,interpolate,compare_rmse,comparison_warnings,
    export_comparison_csv,
)

class TestHmiData(unittest.TestCase):
    def test_force_state_layout_awd_and_motor_torque(self):
        d=force_state_values([120.0,0.1,0.09,2.0,2.1,0.3,2.4,2.5,4.5,4.5])
        self.assertEqual(d["F_cmd_N"],120)
        self.assertAlmostEqual(d["delta_ref_deg"],math.degrees(0.1))
        self.assertAlmostEqual(d["delta_actual_deg"],math.degrees(.09))
        self.assertEqual(d["omega_rl_rad_s"],2.0)
        self.assertEqual(d["omega_fl_rad_s"],2.4)
        self.assertAlmostEqual(d["tau_total_Nm"],18.0)
        self.assertEqual(d["tau_rl_Nm"],4.5)
        self.assertEqual(d["tau_fr_Nm"],4.5)

    def test_missing_state_is_missing_not_fabricated_zero(self):
        state=force_state_values([20.0,0.0])
        self.assertTrue(math.isnan(state["F_cmd_N"]))
        self.assertTrue(math.isnan(state["tau_total_Nm"]))

    def test_quaternion_to_heading_pitch_roll(self):
        roll,pitch,yaw=quaternion_rpy(0.,0.,math.sin(math.pi/4),
                                     math.cos(math.pi/4))
        self.assertAlmostEqual(yaw,math.pi/2,places=6)
        self.assertAlmostEqual(pitch,0.,places=6)
        self.assertAlmostEqual(roll,0.,places=6)

    def test_export_simulink_csv_and_metrics(self):
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)
            matlab=folder/"reference.csv"
            matlab.write_text(
                "t_s,x_rear_m,y_rear_m,v_m_s,yaw_deg,delta_actual_deg,"
                "tau_total_Nm,F_drive_eq_N\n"
                "5,0,0,0,179,0,3,20\n"
                "6,1,0,1,179,10,6,40\n"
                "7,2,0,1,179,10,9,60\n",encoding="utf-8")
            ref=load_matlab_csv(matlab)
            self.assertAlmostEqual(ref[0]["t_s"],0)
            self.assertAlmostEqual(ref[-1]["t_s"],2)
            self.assertAlmostEqual(interpolate(ref,.5,"x_rear_m"),.5)
            self.assertTrue(math.isnan(interpolate(ref,3.,"x_rear_m")))
            robot=[
                {"t_s":0.,"x_rear_m":.1,"y_rear_m":0.,"v_m_s":0.,
                 "yaw_deg":-179.,"delta_actual_deg":0.,"tau_total_Nm":3.,"F_cmd_N":20.},
                {"t_s":1.,"x_rear_m":1.1,"y_rear_m":0.,"v_m_s":1.,
                 "yaw_deg":-179.,"delta_actual_deg":10.,"tau_total_Nm":6.,"F_cmd_N":40.}
            ]
            report=compare_rmse(robot,ref)
            self.assertEqual(report['x_rear_m']['n'],2)
            self.assertAlmostEqual(report['x_rear_m']['rmse'],.1)
            self.assertAlmostEqual(report['yaw_deg']['rmse'],2.)
            p=folder/"comparison.csv"
            export_comparison_csv(p,robot,ref)
            with p.open(encoding="utf-8-sig",newline="") as fp:
                rows=list(csv.DictReader(fp))
            self.assertEqual(len(rows),2)
            self.assertEqual(rows[0]["Gazebo_x_rear_m"],"0.1")
            self.assertAlmostEqual(float(rows[0]["error_yaw_deg"]),2.0)
            out=folder/"gazebo.csv"
            export_csv(out,robot)
            with out.open(encoding="utf-8-sig",newline="") as fp:
                rd=csv.DictReader(fp)
                self.assertEqual(tuple(rd.fieldnames),FIELDS)
                rr=list(rd)
            self.assertEqual(len(rr),2)
            self.assertEqual(rr[0]["x_rear_m"],"0.1")

    def test_matlab_raw_radian_aliases(self):
        with tempfile.TemporaryDirectory() as d:
            file=Path(d)/"matlab.csv"
            file.write_text(
                "Time,xP,yP,v,theta,psi,tau\n"
                "0,1,2,0.2,3.141592653589793,0.1,7.5\n"
                "1,2,2,0.3,3.141592653589793,0.2,5\n",encoding="utf-8")
            ref=load_matlab_csv(file,.15)
            self.assertAlmostEqual(ref[0]['yaw_deg'],180.)
            self.assertAlmostEqual(ref[0]['delta_actual_deg'],math.degrees(.1))
            self.assertAlmostEqual(ref[0]['F_cmd_N'],50.0)

    def test_expected_hmi_topics_manual_authority(self):
        source=(BASE/'tools/hmi_v104.py').read_text(encoding='utf-8')
        self.assertIn("CMD='/ackermann_vehicle/force_steer'",source)
        self.assertIn("STATE='/ackermann_vehicle/force_state'",source)
        self.assertIn("SCENE='/gazebo/model_states'",source)
        self.assertIn("NEARBY='/ackermann_vehicle/nearby_obstacles'",source)
        self.assertIn("self.manual_publisher=None",source)
        self.assertIn("self.node.release_manual()",source)
        self.assertIn("self.node.count_publishers(CMD)>0",source)
        self.assertIn("self.create_subscription(ModelStates,SCENE",source)
        self.assertNotIn("LaserScan",source)
        self.assertIn("self.process.terminate()",source)
        self.assertIn("self.node.send_manual(0.0,0.0)",source)

    def test_matlab_comparison_flags_mismatched_conditions(self):
        rows_g=[{
            't_s':float(i),'x_rear_m':1.0+i,'y_rear_m':0.,
            'v_m_s':.2,'yaw_deg':-179.,'F_cmd_N':90.,
        } for i in range(6)]
        rows_m=[{
            't_s':float(i),'x_rear_m':0.0+i,'y_rear_m':0.,
            'v_m_s':.2,'yaw_deg':179.,'F_cmd_N':25.,
        } for i in range(6)]
        warnings=comparison_warnings(rows_g,rows_m)
        self.assertTrue(any('X ban' in msg for msg in warnings))
        self.assertTrue(any('Lệnh lực' in msg for msg in warnings))
        self.assertFalse(any('yaw' in msg for msg in warnings))

    def test_matlab_comparison_matching_initial_conditions(self):
        rows=[{
            't_s':float(i),'x_rear_m':i*.1,'y_rear_m':0.,
            'v_m_s':.3,'yaw_deg':1.,'F_cmd_N':80.,
        } for i in range(6)]
        self.assertEqual(comparison_warnings(rows,rows),[])

    def test_shortest_route_error(self):
        from hmi_data import distance_to_path
        self.assertTrue(math.isnan(distance_to_path(0,0,[])))
        self.assertAlmostEqual(distance_to_path(3,4,[(0,0),(6,0)]),4)
        self.assertAlmostEqual(distance_to_path(7,0,[(0,0),(6,0)]),1)
        self.assertAlmostEqual(distance_to_path(0,3,[(1,3)]),1)

    def test_matlab_bridge_is_explicit_about_contact_forces(self):
        matlab=(BASE/'matlab/export_ackermann_hmi_csv.m').read_text()
        self.assertIn('StateLog',matlab)
        self.assertIn('CommandLog',matlab)
        self.assertIn('RearForceLog',matlab)
        self.assertIn('Fx_RL_contact_N',matlab)
        self.assertIn('maxTorque=7.5 Nm',matlab)


if __name__=='__main__':
    unittest.main()
