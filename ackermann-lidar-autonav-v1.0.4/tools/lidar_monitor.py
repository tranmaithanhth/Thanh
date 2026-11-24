#!/usr/bin/env python3
"""Sensor-only diagnostic. Subscribes /scan and /odom; NEVER publishes motor force."""
from __future__ import annotations
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from lidar_obstacles import LidarObstacleLayer
from planner_core import Pose
from autonomous_force_nav import yaw_from_quat


class ScanMonitor(Node):
    def __init__(self):
        super().__init__('ackermann_lidar_monitor')
        self.odom=None
        # Use EXACTLY the same terrain, pose and sensor parameters as HMI
        # and autonomous planner. The previous hardcoded config omitted
        # height_shift_gazebo_m, skewing slope classification.
        config=json.loads((Path(__file__).resolve().parents[1]/
                           'mission.json').read_text(encoding='utf-8'))
        self.cfg=config
        self.layer=LidarObstacleLayer(config)
        self.count=0
        self.last_log=0.0
        self.create_subscription(Odometry,'/ackermann_vehicle/odom',self.on_odom,10)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.get_logger().info('[SCAN MONITOR] Listening /scan + /ackermann_vehicle/odom. NO motor publisher.')

    def on_odom(self,msg):
        self.odom=msg

    def on_scan(self,msg):
        self.count+=1
        if self.odom is None:
            if time.monotonic()-self.last_log>1:
                self.get_logger().warn('LaserScan received; waiting for /ackermann_vehicle/odom')
                self.last_log=time.monotonic()
            return
        p=self.odom.pose.pose.position
        yaw=yaw_from_quat(self.odom.pose.pose.orientation)
        rear_offset=float(self.cfg['rear_axle_offset_from_model_origin_m'])
        rear_x=p.x-rear_offset*math.cos(yaw)
        rear_y=p.y-rear_offset*math.sin(yaw)
        q=self.odom.pose.pose.orientation
        model_pose=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not self.layer.update(msg.ranges,msg.angle_min,msg.angle_increment,
                                 msg.range_min,msg.range_max,rear_x,rear_y,yaw,
                                 model_pose=model_pose):
            self.get_logger().warn('Invalid /scan message')
            return
        if time.monotonic()-self.last_log>1.0:
            self.last_log=time.monotonic()
            self.get_logger().info(
                f'[SCAN] packets={self.count} beams={len(msg.ranges)} '
                f'angle=[{math.degrees(msg.angle_min):.0f},{math.degrees(msg.angle_max):.0f}]deg '
                f'front={self.layer.nearest(1):.2f}m rear={self.layer.nearest(-1):.2f}m '
                f'new_obstacles={len(self.layer.world_points())} '
                f'filtered_ground={self.layer.last_filter_stats["terrain"]} '
                f'filtered_self={self.layer.last_filter_stats["self"]} '
                f'filtered_raycast={self.layer.last_filter_stats["ray_ground"]}; '
                f'frame={msg.header.frame_id}')


def main():
    ap=argparse.ArgumentParser()
    ap.parse_args()
    rclpy.init()
    n=ScanMonitor()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
