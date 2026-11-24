#!/usr/bin/env python3
"""Offline tests for HMI-driven Gazebo scene editing (no Qt, ROS or Gazebo)."""
from __future__ import annotations

import math
import subprocess
import sys
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from scene_editor_core import (
    ROBOT_NAME, removable_models, unique_obstacle_name,
    validate_xy, validate_obstacle, obstacle_sdf, ground_center_height,
    conflicts_with_robot, conflicts_with_obstacles,
)
from nearby_scene import NearbyScene, model_collision_radius, sensed_models
from planner_core import terrain


def pose(x,y):
    return types.SimpleNamespace(position=types.SimpleNamespace(x=x,y=y))


class TestSceneEditor(unittest.TestCase):
    def test_editor_can_remove_world_obstacles_not_terrain_or_robot(self):
        names=[
            'obstacle_01','obstacle_04','hmi_obstacle_001_r075',
            'lidar_test_box_v104','test_unknown_box',ROBOT_NAME,
            'start_marker','goal_marker','boundary_north',
            'ackermann_matlab_terrain_v102','sun',
        ]
        result=removable_models(names)
        self.assertEqual(len(result),5)
        self.assertNotIn(ROBOT_NAME,result)
        self.assertNotIn('start_marker',result)
        self.assertNotIn('boundary_north',result)

    def test_obstacle_name_unique_and_radius_is_conservative(self):
        name=unique_obstacle_name([
            'hmi_obstacle_001_r075','hmi_obstacle_002',
            'obstacle_01',
        ],math.sqrt(2)*0.65)
        self.assertEqual(name,'hmi_obstacle_003_r092')
        self.assertAlmostEqual(model_collision_radius(name),0.92)
        self.assertEqual(model_collision_radius('obstacle_01',0.75),0.75)

    def test_cylinder_sdf_includes_real_collision_and_visual(self):
        name='hmi_obstacle_001_r075'
        sdf=obstacle_sdf(name,'cylinder',0.75,1.2)
        root=ET.fromstring(sdf)
        self.assertEqual(root.findtext('model/static'),'true')
        self.assertEqual(root.find('model').get('name'),name)
        self.assertAlmostEqual(float(root.findtext(
            'model/link/collision/geometry/cylinder/radius')),0.75)
        self.assertAlmostEqual(float(root.findtext(
            'model/link/visual/geometry/cylinder/length')),1.2)

    def test_box_sdf_has_matching_physical_and_visual_size(self):
        root=ET.fromstring(obstacle_sdf(
            'hmi_obstacle_008_r142','box',1.0,.85))
        self.assertEqual(root.findtext(
            'model/link/collision/geometry/box/size'),'2.000000 2.000000 0.850000')
        self.assertEqual(root.findtext(
            'model/link/visual/geometry/box/size'),'2.000000 2.000000 0.850000')

    def test_arbitrary_xy_and_auto_height(self):
        x,y=7.2,-2.4
        self.assertEqual(validate_xy(x,y), (x,y))
        self.assertAlmostEqual(
            ground_center_height(x,y,1.2,terrain,0.063),
            terrain(x,y)[0]+.063+.6+.015)

    def test_reject_off_world_nan_and_invalid_shapes(self):
        for x,y in ((20,0),(0,-20),(math.nan,0),(0,math.inf)):
            with self.assertRaises(ValueError):
                validate_xy(x,y)
        with self.assertRaises(ValueError):
            validate_obstacle('sphere',0.5,1.2)
        with self.assertRaises(ValueError):
            validate_obstacle('cylinder',0.,1.)
        with self.assertRaises(ValueError):
            obstacle_sdf('boundary_west','cylinder',0.6,1.2)

    def test_do_not_spawn_obstacle_on_top_of_robot(self):
        self.assertTrue(conflicts_with_robot(1.0,2.0,.6,(1.0,2.0)))
        self.assertFalse(conflicts_with_robot(8.0,2.0,.6,(1.0,2.0)))
        self.assertTrue(conflicts_with_obstacles(
            4.1,.3,1.05,[('obstacle_01',4.1,.3)]))
        self.assertFalse(conflicts_with_obstacles(
            -1.,-2.,1.05,[('obstacle_01',4.1,.3)]))
        # A very large HMI box is assigned its true XY bounding radius;
        # the robot must not be spawned in its collision footprint.
        self.assertTrue(conflicts_with_obstacles(
            0.,0.,1.05,[('hmi_obstacle_002_r354',3.,0.)]))
        self.assertFalse(conflicts_with_obstacles(
            0.,0.,1.05,[('hmi_obstacle_002_r354',10.,0.)]))

    def test_new_object_is_hidden_until_entering_local_sensor(self):
        model='hmi_obstacle_001_r120'
        self.assertEqual(sensed_models([model],[pose(7.,0.)],0.,0.,4.),[])
        seen=sensed_models([model],[pose(5.,0.)],0.,0.,4.)
        self.assertEqual([(n,r) for n,x,y,r in seen],[(model,1.2)])
        self.assertEqual(sensed_models([model],[pose(7.,0.)],0.,0.,4.),[])

    def test_deletion_removes_nearby_ghost_without_waiting_for_ttl(self):
        scene=NearbyScene(radius_m=4.,memory_s=60.)
        name='hmi_obstacle_001_r075'
        scene.ingest([name],[pose(2.,0.)],0.,0.,now=2.)
        self.assertEqual(len(scene.remembered(now=3.)),1)
        scene.ingest([],[],0.,0.,now=4.)
        self.assertEqual(scene.visible,[])
        self.assertEqual(scene.remembered(now=4.),[])

    def test_gazebo_launchers_pass_bash_syntax_check(self):
        # Protect the nested bash -c quoting used by the force robot spawner.
        for script in ('spawn_force_v104.sh','run_world_v104.sh',
                       'auto_drive_v104.sh','hmi_v104.sh'):
            result=subprocess.run(
                ['bash','-n',str(ROOT/script)],
                capture_output=True,text=True,check=False)
            self.assertEqual(result.returncode,0,msg=script+': '+result.stderr)

    def test_hmi_integrates_gazebo_services_and_robot_spawn(self):
        hmi=(ROOT/'tools/hmi_v104.py').read_text(encoding='utf-8')
        script=(ROOT/'spawn_force_v104.sh').read_text(encoding='utf-8')
        self.assertIn("self.create_client(SpawnEntity,'/spawn_entity')",hmi)
        self.assertIn("self.create_client(DeleteEntity,'/delete_entity')",hmi)
        self.assertIn("self.tabs.addTab(scroll,'Quản lý mô phỏng')",hmi)
        self.assertIn("self._scene_add_obstacle",hmi)
        self.assertIn("self._scene_spawn_robot",hmi)
        self.assertIn("self._scene_delete_robot",hmi)
        self.assertIn("self._scene_delete_all",hmi)
        self.assertIn("self.map_panel.canvas.mpl_connect('button_press_event'",hmi)
        self.assertIn("ACKERMANN_SPAWN_X",hmi)
        self.assertIn("ACKERMANN_SPAWN_Y",hmi)
        self.assertIn("ACKERMANN_SPAWN_YAW",hmi)
        self.assertIn("Publisher count:",script)
        self.assertNotIn("ros2 topic list | grep -q '^/ackermann_vehicle/odom$'",script)


if __name__=='__main__':
    unittest.main()
