#!/usr/bin/env python3
"""Controlled Ackermann stage-3 reverse/forward diagnostic (no sensors).

Only run in a *fresh* terrain world with one stage-3 robot at (0,-2).
Publishes cmd_vel and observes odometry from the existing Gazebo plugin.
Exits if a second cmd_vel publisher is found or odom stops updating.
"""
from __future__ import annotations
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD = '/ackermann_vehicle/cmd_vel'
ODOM = '/ackermann_vehicle/odom'


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class ReverseProbe(Node):
    def __init__(self):
        super().__init__('ackermann_reverse_probe')
        self.pub = self.create_publisher(Twist, CMD, 10)
        self.sub = self.create_subscription(Odometry, ODOM, self.on_odom, 10)
        self.odom = None
        self.last_odom_wall = 0.0
        self.last_sim = None
        self.odom_count = 0

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            return
        self.odom = msg
        self.last_odom_wall = time.monotonic()
        self.last_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_count += 1

    def cmd(self, speed):
        message = Twist()
        message.linear.x = float(speed)
        message.angular.z = 0.0
        self.pub.publish(message)

    def wait_odom(self, max_wait=12):
        t0 = time.monotonic()
        while rclpy.ok() and self.odom_count < 3 and time.monotonic() - t0 < max_wait:
            self.cmd(0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom_count < 3:
            raise RuntimeError('Missing /ackermann_vehicle/odom; check stage 3 and Gazebo Play')

    def check_unique_cmd_publisher(self):
        # Allow graph discovery to settle; this node is itself one publisher.
        rclpy.spin_once(self, timeout_sec=0.5)
        count = self.count_publishers(CMD)
        if count != 1:
            raise RuntimeError(f'{count} publishers detected on {CMD}; stop stage3 console and all other controllers')
        if self.count_subscribers(CMD) < 1:
            raise RuntimeError('No subscriber to cmd_vel; Ackermann Drive plugin is not listening')

    def sample(self):
        p = self.odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z),
                yaw_from_quat(self.odom.pose.pose.orientation),
                self.last_sim)

    def phase(self, name, speed, duration):
        t0 = time.monotonic()
        start = self.sample()
        while time.monotonic() - t0 < duration and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.07)
            if time.monotonic() - self.last_odom_wall > 1.5:
                raise RuntimeError(f'Odometry STALE during {name}; Gazebo may be paused/frozen')
            if self.count_publishers(CMD) != 1:
                raise RuntimeError(f'Another cmd_vel publisher appeared during {name}')
            # Guard against leaving the flat, obstacle-free diagnostic corridor.
            x, y, _, _, _ = self.sample()
            if not (-3.0 < x < 1.5 and -4.0 < y < -1.0):
                raise RuntimeError(f'Left safe diagnostic corridor at (x={x:.2f}, y={y:.2f})')
            self.cmd(speed)
        finish = self.sample()
        elapsed = time.monotonic() - t0
        dx, dy = finish[0] - start[0], finish[1] - start[1]
        along = dx * math.cos(start[3]) + dy * math.sin(start[3])
        sim_advance = finish[4] - start[4]
        ratio = along / (speed * elapsed) if abs(speed) > 0.01 else None
        print(f'[{name}] cmd={speed:+.2f} m/s | elapsed={elapsed:.1f}s '
              f'| Δlong={along:+.3f}m | Δx={dx:+.3f} Δy={dy:+.3f}m '
              f'| odom_v={self.odom.twist.twist.linear.x:+.3f}m/s '
              f'| sim_advance={sim_advance:.2f}s '
              + (f'| achieved_ratio={ratio:.0%}' if ratio is not None else ''), flush=True)
        return (name, speed, along, elapsed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase-seconds', type=float, default=5.0,
                        help='Duration of each powered phase in wall seconds (2..8)')
    args = parser.parse_args()
    if not (2 <= args.phase_seconds <= 8):
        parser.error('--phase-seconds must be between 2 and 8')
    rclpy.init()
    node = ReverseProbe()
    results = []
    try:
        print('[CHECK] Run in a NEW Gazebo world with ONLY stage 3 at x=0 y=-2.', flush=True)
        print('[CHECK] Do not run stage3_control_feedback.sh or any other cmd_vel publisher.', flush=True)
        node.wait_odom()
        node.check_unique_cmd_publisher()
        x, y, z, yaw, _ = node.sample()
        if not (-0.6 < x < 0.6 and -3.0 < y < -1.0):
            raise RuntimeError(f'Spawn must be near x=0,y=-2. Current x={x:.2f}, y={y:.2f}; restart Gazebo.')
        print(f'[START] pose x={x:.3f} y={y:.3f} z={z:.3f} yaw={math.degrees(yaw):.2f} deg',flush=True)
        phases = [
            ('REVERSE_010', -0.10, args.phase_seconds),
            ('BRAKE_A', 0.0, 1.0),
            ('REVERSE_025', -0.25, args.phase_seconds),
            ('BRAKE_B', 0.0, 1.0),
            ('FORWARD_025', +0.25, args.phase_seconds),
            ('BRAKE_END', 0.0, 1.0),
        ]
        for name, speed, duration in phases:
            results.append(node.phase(name, speed, duration))
        print('\n[RESULT] Compare signed displacement along car heading:', flush=True)
        for name, speed, displacement, duration in results:
            if abs(speed) < 0.01:
                continue
            expected = speed * duration
            print(f'  {name:15s} desired≈{expected:+.2f}m observed={displacement:+.3f}m',flush=True)
        print('If forward works but both reverse phases fail, inspect reverse drive PID/joints/contacts.',flush=True)
    except (KeyboardInterrupt, RuntimeError) as exc:
        print(f'[STOP] {exc or "Ctrl+C"}',flush=True)
    finally:
        for _ in range(10):
            node.cmd(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()