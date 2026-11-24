#!/usr/bin/env python3
"""ROS-free regression tests for v1.0.4 default Nearby, plus legacy LiDAR.

Default navigation is NOT LiDAR: no preloaded obstacle coordinates, nearby
Gazebo ModelStates, and unchanged 20kg AWD force plugin. Legacy LiDAR filters
remain independently tested as optional regression coverage.
"""
from __future__ import annotations
import json
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'tools'))
from lidar_obstacles import LidarObstacleLayer,stopping_distance
from lidar_planner import DynamicMission
from planner_core import Pose
from patch_robot_v104 import patch


class TestLidarV104(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((BASE/'mission.json').read_text())

    def test_scan_geometry_and_timeout(self):
        layer=LidarObstacleLayer(self.cfg)
        ranges=[math.inf]*91
        # beam45 = 0deg. From rear P=(0,-2), lidar is at X=.60m.
        ranges[45]=2.4
        self.assertTrue(layer.update(ranges,-math.pi,2*math.pi/90,.15,8.,
                                     0.0,-2.0,0.0,now=10.0))
        self.assertAlmostEqual(layer.nearest(1,now=10.2),2.4)
        hits=layer.world_points(now=10.2)
        self.assertTrue(any(abs(x-3.0)<0.1 and abs(y+2.0)<0.1 for x,y in hits))
        self.assertEqual(layer.world_points(now=12.0),[])
        self.assertTrue(math.isnan(layer.nearest(1,now=12.0)))

    def test_rear_scan_for_reverse_ackermann(self):
        layer=LidarObstacleLayer(self.cfg)
        ranges=[math.inf]*91
        ranges[0]=2.0                 # -180deg, behind the robot
        layer.update(ranges,-math.pi,2*math.pi/90,.15,8.,
                     0.0,-2.0,math.pi,now=10.0)
        self.assertAlmostEqual(layer.nearest(-1,now=10.0),2.0)

    def test_ground_echo_rejected_when_beam_hits_known_terrain(self):
        layer=LidarObstacleLayer(self.cfg)
        ranges=[math.inf]*91
        ranges[45]=1.65
        # Body pitched +0.2rad: a forward beam hits near-ground. A ground
        # return must not be inserted as a dynamic box or trigger braking.
        q=(0.0,math.sin(0.1),0.0,math.cos(0.1))
        layer.update(ranges,-math.pi,2*math.pi/90,.15,8.,0,-2,0,
                     now=10.0,model_pose=(0.0,-2.0,0.063,*q))
        self.assertEqual(layer.world_points(now=10.0),[])
        self.assertTrue(math.isinf(layer.nearest(1,now=10.0)))

    def test_sensor_data_qos_best_effort(self):
        for name in ('autonomous_lidar_nav.py','lidar_monitor.py'):
            src=(BASE/'tools'/name).read_text()
            self.assertIn('qos_profile_sensor_data',src)
            self.assertIn('model_pose=model_pose',src)

    def test_legacy_known_static_obstacle_not_duplicated_as_dynamic(self):
        # Compatibility test for the optional LEGACY LiDAR filter only.
        # v1.0.4 default must NOT preload this position into its A* map.
        legacy_cfg=dict(self.cfg)
        legacy_cfg['obstacles_xy_radius']=[(4.1,0.3,0.75)]
        layer=LidarObstacleLayer(legacy_cfg)
        # lidar origin=(0.6,-2); hit at (4.1,0.3) is a known fixed cylinder
        bearing=math.atan2(2.3,3.5)
        i=round((bearing+math.pi)/(2*math.pi/90))
        ranges=[math.inf]*91
        ranges[i]=math.hypot(3.5,2.3)
        layer.update(ranges,-math.pi,2*math.pi/90,.15,8.,0,-2,0,now=10)
        self.assertEqual(layer.world_points(now=10),[])

    def test_invalid_scan_does_not_claim_ready(self):
        layer=LidarObstacleLayer(self.cfg)
        self.assertFalse(layer.update([],0.0,0.0,0.1,8.,0,0,0,now=3))
        self.assertFalse(layer.ready(now=3))

    def test_brake_margin_rises_with_speed(self):
        d1=stopping_distance(.2,1.4,.3,.85)
        d2=stopping_distance(1.0,1.4,.3,.85)
        self.assertGreater(d2,d1)
        self.assertGreaterEqual(d1,.85)

    def test_new_obstacle_blocks_old_global_route(self):
        m=DynamicMission(self.cfg)
        route=m.astar((-.4,-2.0))
        self.assertEqual(route[0],(-.4,-2.))
        self.assertEqual(route[-1],(16.,.6))
        m.update_dynamic([(8.2,-2.2)])
        self.assertTrue(m.path_blocked((1.0,-2.0)))
        new_route=m.astar((-.4,-2.0))
        self.assertEqual(new_route[-1],(16.,.6))
        self.assertFalse(m.path_blocked((-.4,-2.0)))

    def test_dynamic_obstacle_near_start_avoids_collision(self):
        m=DynamicMission(self.cfg)
        m.update_dynamic([(0.0,-2.0)])
        self.assertFalse(m.safe_xy(0.0,-2.0))
        with self.assertRaises(ValueError):
            m.astar((0.0,-2.0))

    def test_no_lidar_in_default_urdf_keeps_20kg_force_plugin(self):
        cfg=json.loads((BASE/'matlab_vehicle_params.json').read_text())
        root=ET.Element('robot',{'name':'ackermann_vehicle'})
        for name in ('chassis','rear_left_wheel','rear_right_wheel',
                     'front_left_wheel','front_right_wheel',
                     'front_left_steering_link','front_right_steering_link'):
            link=ET.SubElement(root,'link',{'name':name})
            inertial=ET.SubElement(link,'inertial')
            ET.SubElement(inertial,'mass',{'value':'1'})
            ET.SubElement(inertial,'inertia',{'ixx':'0.1','iyy':'0.1','izz':'0.1'})
        with tempfile.TemporaryDirectory() as d:
            src,dst=Path(d)/'stage3.urdf',Path(d)/'force_nearby.urdf'
            ET.ElementTree(root).write(src)
            # Exactly the same option that spawn_force_v104.sh now passes.
            mass=patch(src,dst,cfg,use_lidar=False)
            urdf=ET.parse(dst).getroot()
            self.assertAlmostEqual(mass,20.0,places=6)
            self.assertEqual(len(urdf.findall("link[@name='lidar_link']")),0)
            self.assertEqual(len(urdf.findall("joint[@name='lidar_fixed_joint']")),0)
            self.assertEqual(len(urdf.findall(".//sensor[@name='lidar_2d']")),0)
            self.assertEqual(len(urdf.findall('.//plugin[@name="gazebo_ros_ray_sensor_lidar_v104"]')),0)
            plugins=urdf.findall('.//plugin[@name="ackermann_force_drive"]')
            self.assertEqual(len(plugins),1)
            plugin=plugins[0]
            self.assertEqual(plugin.get('filename'),'libackermann_force_drive.so')
            self.assertAlmostEqual(float(plugin.findtext('max_force')),
                                   float(cfg['max_drive_force_N']))
            self.assertEqual(plugin.findtext('drive_mode'),cfg['drive_mode'])

    def test_launcher_and_safety_guards(self):
        run=(BASE/'run_world_v104.sh').read_text()
        spawn=(BASE/'spawn_force_v104.sh').read_text()
        auto=(BASE/'auto_drive_v104.sh').read_text()
        nav=(BASE/'tools/autonomous_nearby_nav.py').read_text()
        cfg=json.loads((BASE/'mission.json').read_text())
        # Check actual default execution path; never assert stale LiDAR code.
        self.assertIn('libgazebo_ros_state.so',run)
        self.assertIn('prepare_proximity_world.py',run)
        self.assertIn('patch_robot_v104.py',spawn)
        self.assertIn('--no-lidar',spawn)
        self.assertIn('autonomous_nearby_nav.py',auto)
        self.assertNotIn('autonomous_lidar_nav.py',auto)
        self.assertIn('ModelStates,SCENE_TOPIC',nav)
        self.assertIn("SCENE_TOPIC='/gazebo/model_states'",nav)
        self.assertNotIn('LaserScan',nav)
        self.assertEqual(cfg['obstacles_xy_radius'],[])
        self.assertIn("cfg['obstacles_xy_radius']=[]",nav)
        # Preserve fail-safe behavior: stale scene data => zero/brake;
        # dynamic obstacles => replan, or hold if too close.
        self.assertIn('self.scene.age(now)>',nav)
        self.assertIn('nearest_clearance(',nav)
        self.assertIn('self.mission.update_dynamic(self.scene.remembered(now))',nav)
        self.assertIn('self.mission.path_blocked',nav)
        self.assertIn('self.mission.astar((pose.x,pose.y))',nav)
        self.assertIn('self.braking_command(pose)',nav)
        self.assertIn('multiple force_steer publishers',nav)


if __name__=='__main__':unittest.main()
