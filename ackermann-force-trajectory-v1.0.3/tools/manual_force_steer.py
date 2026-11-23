#!/usr/bin/env python3
"""Keyboard manual force [N] / equivalent steering [rad], ROS 2 Humble."""
import argparse
import json
from pathlib import Path
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class ManualForce(Node):
    def __init__(self,args):
        super().__init__('ackermann_manual_force_v103')
        self.args=args
        self.pub=self.create_publisher(Float64MultiArray,'/ackermann_vehicle/force_steer',10)
        self.sub=self.create_subscription(Odometry,'/ackermann_vehicle/odom',self.on_odom,10)
        self.F=0.0;self.delta=0.0;self.last_key=time.monotonic();self.last_odom=0.
        self.odom=None;self.exit=False;self.start=time.monotonic();self.last_print=0.
        print('[KEYS] W: +force; S: -force (through zero); A/D: steer; C: center; X/Space: zero; Q: exit')
        print(f'[LIMITS] F +/-{args.max_force:.1f}N, steer +/-{args.max_steer:.2f}rad, deadman {args.deadman}s')

    def on_odom(self,msg):
        self.odom=msg;self.last_odom=time.monotonic()

    def zero(self):
        self.F=0.;self.delta=0.

    def key(self,k):
        if k in ('q','\x03'):
            self.zero();self.exit=True;return
        if k not in 'wsadcx ':
            return
        self.last_key=time.monotonic()
        if k in 'ws':
            direction=1 if k=='w' else -1
            f=self.F+direction*self.args.force_step
            if self.F*f<0:f=0.
            self.F=round(max(-self.args.max_force,min(self.args.max_force,f)),3)
        elif k in 'ad':
            self.delta=round(max(-self.args.max_steer,min(self.args.max_steer,
                             self.delta+(self.args.steer_step if k=='a' else -self.args.steer_step))),4)
        elif k=='c':self.delta=0.
        else:self.zero()
        print(f'\n[SETPOINT] F={self.F:+.1f} N | steer={math.degrees(self.delta):+.1f} deg')

    def send(self):
        msg=Float64MultiArray();msg.data=[self.F,self.delta];self.pub.publish(msg)

    def tick(self):
        now=time.monotonic()
        if now-self.start>2 and self.count_publishers('/ackermann_vehicle/force_steer')!=1:
            print('[ERROR] Conflicting force_steer publisher! Stop other controller.')
            self.zero();self.exit=True;return
        if now-self.last_key>self.args.deadman or now-self.last_odom>1.2:
            if self.F or self.delta:
                print('\n[SAFETY] Command timed out or odometry stale -> force=0; steer=0')
            self.zero()
        self.send()
        if now-self.last_print>.5:
            self.last_print=now
            if self.odom and now-self.last_odom<1.2:
                p=self.odom.pose.pose.position;v=self.odom.twist.twist.linear.x
                print(f'[POSE] XYZ=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) v={v:+.3f}m/s '
                      f'F={self.F:+.1f}N delta={math.degrees(self.delta):+.1f}deg',flush=True)
            else:print('[POSE] Awaiting odom from v1.0.3 force plugin...',flush=True)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--params',type=Path,default=Path(__file__).resolve().parents[1]/'matlab_vehicle_params.json')
    ap.add_argument('--force-step',type=float,default=20.)
    ap.add_argument('--steer-step',type=float,default=0.05)
    ap.add_argument('--max-force',type=float,default=None,help='Optional cap, never above force limit in params JSON')
    ap.add_argument('--max-steer',type=float,default=.52)
    ap.add_argument('--deadman',type=float,default=5.)
    a=ap.parse_args()
    limits=json.loads(a.params.read_text(encoding='utf-8'))
    profile_force=float(limits['max_drive_force_N'])
    if a.max_force is None:
        a.max_force=profile_force
    if not (0<a.force_step<=a.max_force<=profile_force+1e-6) or not(0<a.steer_step<=a.max_steer<=.52) or a.deadman<=0:
        ap.error('Invalid force/steering/rate limits')
    if not sys.stdin.isatty():ap.error('Needs interactive Terminal')
    rclpy.init();node=ManualForce(a)
    original=termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not node.exit:
            rclpy.spin_once(node,timeout_sec=.03)
            if select.select([sys.stdin],[],[],0)[0]:
                for k in os.read(sys.stdin.fileno(),128).decode('utf-8','ignore').lower():
                    node.key(k)
            node.tick()
    except KeyboardInterrupt:pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,original)
        node.zero()
        if node.count_publishers('/ackermann_vehicle/force_steer')==1:
            for _ in range(6):node.send();rclpy.spin_once(node,timeout_sec=.03)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()