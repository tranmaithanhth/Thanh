#!/usr/bin/env python3
"""Offline tests for the NEW bounded Gazebo-world nearby sensor (no ROS needed)."""
from pathlib import Path
import json
import math
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))

from nearby_scene import (
    NearbyScene,sensed_models,closest_edge_distance,nearest_clearance,
)
from lidar_planner import DynamicMission
from prepare_proximity_world import prepare_world


def pose(x,y):
    return types.SimpleNamespace(position=types.SimpleNamespace(x=x,y=y))


class TestUnknownNearby(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'mission.json').read_text(encoding='utf-8'))
        self.names=[
            'sun','ackermann_matlab_terrain_v102','start_marker',
            'goal_marker','boundary_east','ackermann_force_nearby_v104',
            'obstacle_01','obstacle_02','test_unknown_box',
        ]
        self.poses=[
            pose(0.,0.),pose(0.,0.),pose(0.,0.),pose(16.,.6),
            pose(19.5,0.),pose(0.,-2.),
            pose(4.1,0.3),pose(7.3,1.15),pose(9.,-2.),
        ]

    def test_planner_starts_without_global_obstacle_coordinates(self):
        self.assertEqual(self.cfg['obstacles_xy_radius'],[])
        mission=DynamicMission(self.cfg)
        self.assertEqual(mission.obstacles,[])
        self.assertEqual(mission.dynamic_obstacles,[])

    def test_scene_does_not_reveal_models_outside_four_meters(self):
        seen=sensed_models(self.names,self.poses,0.,-2.,4.,.75)
        names=[p[0] for p in seen]
        self.assertIn('obstacle_01',names)
        self.assertNotIn('obstacle_02',names)
        self.assertNotIn('test_unknown_box',names)
        self.assertNotIn('ackermann_force_nearby_v104',names)
        self.assertNotIn('start_marker',names)

    def test_reveal_when_robot_moves_closer_and_remember_after_leaving(self):
        scene=NearbyScene(radius_m=4.,default_radius_m=.75,memory_s=10.)
        self.assertEqual(scene.ingest(self.names,self.poses,-5.,-2.,now=2.),[])
        near=scene.ingest(self.names,self.poses,0.,-2.,now=3.)
        self.assertEqual([p[0] for p in near],['obstacle_01'])
        scene.ingest(self.names,self.poses,-5.,-2.,now=4.)
        self.assertEqual(len(scene.visible),0)
        self.assertEqual(len(scene.remembered(now=4.)),1)
        self.assertEqual(len(scene.remembered(now=14.)),0)

    def test_inside_radius_boundary_includes_obstacle_edge(self):
        self.assertAlmostEqual(closest_edge_distance(4.5,0.,.75,0.,0.),3.75)
        self.assertEqual(len(sensed_models(['unknown'],[pose(4.5,0.)],
                                           0.,0.,4.,.75)),1)
        self.assertEqual(len(sensed_models(['unknown'],[pose(5.,0.)],
                                           0.,0.,4.,.75)),0)

    def test_only_forward_collision_corridor_triggers_forward_brake(self):
        obstacles=[
            ('ahead',2.2,0.,.75),
            ('left',1.,3.2,.75),
        ]
        self.assertAlmostEqual(
            nearest_clearance(obstacles,0.,0.,0.,1),2.2-.75-.8)
        self.assertTrue(math.isinf(nearest_clearance(
            [('left',1.,3.2,.75)],0.,0.,0.,1)))
        self.assertTrue(math.isinf(nearest_clearance(
            [('ahead',2.2,0.,.75)],0.,0.,0.,-1)))

    def test_packet_only_visible_obstacles_and_validation(self):
        scene=NearbyScene(radius_m=4.)
        scene.ingest(['obstacle_A'],[pose(2.,1.)],0.,0.,now=2.)
        self.assertEqual(scene.packet(),[2.,1.,.75])
        self.assertEqual(NearbyScene.unpack_packet(scene.packet()),[(2.,1.,.75)])
        with self.assertRaises(ValueError):
            NearbyScene.unpack_packet([1.,2.])
        with self.assertRaises(ValueError):
            NearbyScene.unpack_packet([1.,2.,-1.])

    def test_world_plugin_is_valid_and_preserves_collision_models(self):
        source=ROOT/'worlds/ackermann_matlab_terrain_walled.world'
        with tempfile.TemporaryDirectory() as d:
            output=Path(d)/'world.world'
            prepare_world(source,output)
            doc=ET.parse(output).getroot()
            world=doc.find('world')
            self.assertIsNotNone(world)
            self.assertEqual(len(world.findall('model')),6+4)
            plugin=world.find("plugin[@name='ackermann_proximity_world_state']")
            self.assertIsNotNone(plugin)
            self.assertEqual(plugin.get('filename'),'libgazebo_ros_state.so')
            self.assertEqual(plugin.findtext('ros/namespace'),'/gazebo')
            self.assertEqual(plugin.findtext('ros/remapping'),
                             'model_states:=/gazebo/model_states')

    def test_actual_autonav_launch_uses_nearby_not_lidar(self):
        launch=(ROOT/'auto_drive_v104.sh').read_text()
        nav=(ROOT/'tools/autonomous_nearby_nav.py').read_text()
        hmi=(ROOT/'tools/hmi_v104.py').read_text()
        self.assertIn('autonomous_nearby_nav.py',launch)
        self.assertIn("cfg['obstacles_xy_radius']=[]",nav)
        self.assertIn("SCENE_TOPIC='/gazebo/model_states'",nav)
        self.assertNotIn('LaserScan',nav)
        self.assertNotIn('SCAN=',hmi)
        self.assertNotIn('LaserScan',hmi)
        self.assertNotIn("for x,y,r in self.cfg['obstacles_xy_radius']",hmi)


if __name__=='__main__':
    unittest.main()
