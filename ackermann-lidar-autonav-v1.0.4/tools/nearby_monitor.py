#!/usr/bin/env python3
"""Gazebo nearby sensing monitor; NEVER publishes drive force or scans."""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from nearby_scene import NearbyScene


class Monitor(Node):
    def __init__(self):
        super().__init__('ackermann_nearby_monitor')
        cfg=json.loads((Path(__file__).resolve().parents[1]/'mission.json').read_text())
        self.radius=float(cfg.get('nearby_radius_m',4.0))
        self.default_radius=float(cfg.get('nearby_obstacle_radius_m',0.75))
        self.rear_offset=float(cfg['rear_axle_offset_from_model_origin_m'])
        self.scene=NearbyScene(self.radius,self.default_radius)
        self.odom=None
        self.last_print=0.0
        self.count=0
        self.create_subscription(Odometry,'/ackermann_vehicle/odom',self.on_odom,10)
        self.create_subscription(ModelStates,'/gazebo/model_states',self.on_scene,10)
        self.get_logger().info('[NEARBY MONITOR] Gazebo models only. NO /scan. NO force publisher.')

    def on_odom(self,msg):
        self.odom=msg

    def on_scene(self,msg):
        self.count+=1
        if self.odom is None:
            return
        p=self.odom.pose.pose.position
        q=self.odom.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x=p.x-self.rear_offset*math.cos(yaw)
        y=p.y-self.rear_offset*math.sin(yaw)
        seen=self.scene.ingest(msg.name,msg.pose,x,y)
        now=time.monotonic()
        if now-self.last_print>=1.0:
            self.last_print=now
            self.get_logger().info(
                '[NEARBY] packets=%d visible=%d learned=%d within=%.1fm names=%s' %
                (self.count,len(seen),len(self.scene.remembered()),
                 self.radius,[n for n,_,_,_ in seen]))


def main():
    rclpy.init()
    node=Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
