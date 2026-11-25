#!/usr/bin/env python3
"""Read-only ROS2 verification: 2 rear drive wheels, zero front motor torque.

Run after HMI/manual applies a small nonzero force, e.g. +40 N.
NEVER publishes commands; this tool cannot start or move the robot.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Monitor(Node):
    def __init__(self, force_threshold):
        super().__init__('ackermann_rwd_drive_check')
        self.threshold=force_threshold
        self.seen=0
        self.result=None
        self.subscription=self.create_subscription(
            Float64MultiArray,'/ackermann_vehicle/force_state',self.on_state,10)

    def on_state(self,msg):
        if len(msg.data)<10:
            self.result=(False,'force_state requires at least 10 numbers')
            return
        vals=[float(v) for v in msg.data[:10]]
        if not all(math.isfinite(x) for x in vals):
            return
        f,delta_ref,delta_actual,omega_rl,omega_rr,v_body,omega_fl,omega_fr,tau_rear,tau_front=vals
        if abs(f)<self.threshold:
            return
        self.seen+=1
        # Plugin v1.0.4: 0.15m wheel radius, split TOTAL torque in half.
        expected_each=f*0.15/2.
        rear_ok=math.isclose(tau_rear,expected_each,abs_tol=.025)
        front_ok=abs(tau_front)<1e-5
        ok=rear_ok and front_ok
        self.result=(ok,
            f'F={f:+.2f}N | tau_RL=tau_RR={tau_rear:+.3f}Nm '
            f'(expected {expected_each:+.3f}Nm) | '
            f'tau_FL=tau_FR={tau_front:+.3f}Nm (expected 0) | '
            f'steer_ref={math.degrees(delta_ref):+.1f}deg '
            f'steer_measured={math.degrees(delta_actual):+.1f}deg | '
            f'omega_RL={omega_rl:+.2f},RR={omega_rr:+.2f},'
            f'FL={omega_fl:+.2f},FR={omega_fr:+.2f} rad/s')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout',type=float,default=15.0)
    p.add_argument('--min-force',type=float,default=5.0)
    args=p.parse_args()
    if args.timeout<=0 or args.min_force<=0:p.error('positive thresholds required')
    rclpy.init()
    mon=Monitor(args.min_force)
    try:
        end=time.monotonic()+args.timeout
        while rclpy.ok() and mon.result is None and time.monotonic()<end:
            rclpy.spin_once(mon,timeout_sec=.1)
        if mon.result is None:
            print('INCONCLUSIVE: no nonzero force command observed. '
                  'Select HMI Manual and apply a small +40N command.')
            return 2
        ok,message=mon.result
        print(('PASS RWD: ' if ok else 'FAIL RWD: ')+message)
        return 0 if ok else 1
    finally:
        mon.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
