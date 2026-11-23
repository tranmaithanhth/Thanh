#!/usr/bin/env python3
"""Ackermann Gazebo Classic stage-3 only: keyboard drive + world-pose feedback.

This does not launch Gazebo, change the robot, or add LiDAR / cameras.
Uses the existing gazebo_ros_ackermann_drive Odometry publisher. The plugin
gets its pose via model_->WorldPose() and publishes it under odom frame ID.
"""
from __future__ import annotations
import argparse
import csv
import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

ODOM = '/ackermann_vehicle/odom'
CMD = '/ackermann_vehicle/cmd_vel'
POSE = '/ackermann_vehicle/pose_feedback'
RPY = '/ackermann_vehicle/rpy_deg'


def euler_deg(quat) -> tuple[float, float, float]:
    """Standard aerospace roll / pitch / yaw, intrinsic XYZ convention."""
    x, y, z, w = (float(quat.x), float(quat.y), float(quat.z), float(quat.w))
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = (n / norm for n in (x, y, z, w))
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class Stage3Console(Node):
    def __init__(self, args):
        super().__init__('ackermann_stage3_console')
        self.args = args
        self.odom = None
        self.received_at = None
        self.angles = (float('nan'),) * 3
        self.commanded_speed = 0.0
        self.commanded_yaw_rate = 0.0
        self.last_key_at = time.monotonic()
        self.last_report_at = 0.0
        self.last_csv_at = 0.0
        self.last_pose_pub_at = 0.0
        self.last_missing_at = 0.0
        self.stale_warning = False
        self.stall_since = None
        self.last_stall_report = 0.0
        self.exit_requested = False
        self.odom_sub = self.create_subscription(Odometry, ODOM, self.on_odom, 10)
        self.pose_pub = self.create_publisher(PoseStamped, POSE, 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, RPY, 10)
        self.cmd_pub = None if args.monitor_only else self.create_publisher(Twist, CMD, 10)
        self.csv_file = None
        self.csv_writer = None
        if args.csv:
            path = Path(args.csv).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            append = path.exists() and path.stat().st_size > 0
            self.csv_file = path.open('a', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            if not append:
                self.csv_writer.writerow([
                    'sim_time_sec', 'x_m', 'y_m', 'z_m', 'roll_deg',
                    'pitch_deg', 'yaw_deg', 'linear_x_mps', 'angular_z_radps',
                    'cmd_linear_x_mps', 'cmd_angular_z_radps'])
            print(f'[LOG] CSV: {path}', flush=True)
        print(f'[ROS] Subscribing {ODOM}', flush=True)
        print(f'[ROS] Feedback topics {POSE} and {RPY} (x=roll, y=pitch, z=yaw in degrees)', flush=True)
        if args.monitor_only:
            print('[MODE] Monitor-only: no speed commands will be sent.', flush=True)
        else:
            print(f'[KEYS] w: forward +0.25 | s: reverse -{args.reverse_speed:.2f} | a: turn left | d: turn right', flush=True)
            print('       c: straighten | x/SPACE: stop | q: stop and exit', flush=True)
            print(f'[SAFETY] No LiDAR: automatic stop after {args.deadman:.1f} s without a key; '
                  f'zero command if odom is stale for {args.stale:.1f} s.', flush=True)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        values = [p.x, p.y, p.z, msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        if not all(math.isfinite(float(v)) for v in values):
            return
        try:
            angles = euler_deg(msg.pose.pose.orientation)
        except ValueError:
            return
        self.odom = msg
        self.angles = angles
        self.received_at = time.monotonic()
        if self.received_at - self.last_pose_pub_at < 0.2:
            return
        self.last_pose_pub_at = self.received_at
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)
        rpy = Vector3Stamped()
        rpy.header = msg.header
        rpy.vector.x, rpy.vector.y, rpy.vector.z = angles
        self.rpy_pub.publish(rpy)

    def zero(self):
        self.commanded_speed = 0.0
        self.commanded_yaw_rate = 0.0

    def key(self, ch: str):
        if ch == 'q' or ch == '\x03':
            self.zero()
            self.exit_requested = True
            return
        if ch not in ('w', 's', 'a', 'd', 'c', 'x', ' '):
            return
        self.last_key_at = time.monotonic()
        if ch == 'w':
            self.commanded_speed = 0.25
        elif ch == 's':
            self.commanded_speed = -self.args.reverse_speed
        elif ch == 'a':
            self.commanded_yaw_rate = 0.25
        elif ch == 'd':
            self.commanded_yaw_rate = -0.25
        elif ch == 'c':
            self.commanded_yaw_rate = 0.0
        else:
            self.zero()

    def publish_cmd(self, speed: float, yaw_rate: float):
        if self.cmd_pub is None:
            return
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(yaw_rate)
        self.cmd_pub.publish(cmd)

    def tick(self):
        now = time.monotonic()
        fresh = self.received_at is not None and now - self.received_at < self.args.stale
        active = fresh and now - self.last_key_at < self.args.deadman
        if not active and (self.commanded_speed != 0.0 or self.commanded_yaw_rate != 0.0):
            print('\n[SAFETY] Stale odometry or keyboard timeout: STOP', flush=True)
            self.zero()
        if not self.args.monitor_only:
            self.publish_cmd(self.commanded_speed if active else 0.0,
                             self.commanded_yaw_rate if active else 0.0)
        # Diagnostic only: do not alter motion or contact physics.
        # Warn if a continuous nonzero speed request fails to move the chassis.
        if (fresh and active and self.odom is not None
                and abs(self.commanded_speed) >= 0.08
                and abs(self.odom.twist.twist.linear.x) < 0.025):
            if self.stall_since is None:
                self.stall_since = now
            if now - self.stall_since >= 4.0 and now - self.last_stall_report >= 4.0:
                p = self.odom.pose.pose.position
                print(f'[WARN] STALL: commanded {self.commanded_speed:+.2f} m/s '
                      f'but odom v={self.odom.twist.twist.linear.x:+.3f} m/s '
                      f'for >=4 s at X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f}. '
                      'Check wheel contact/friction and other cmd_vel publishers.', flush=True)
                self.last_stall_report = now
        else:
            self.stall_since = None
        if now - self.last_report_at >= 0.5:
            self.last_report_at = now
            if self.odom is None:
                print('[POSE] Waiting for odometry from stage-3 robot ...', flush=True)
            elif not fresh:
                print('[POSE] STALE: gazebo paused/frozen or odom not arriving', flush=True)
            else:
                p = self.odom.pose.pose.position
                v = self.odom.twist.twist
                roll, pitch, yaw = self.angles
                print(f'[POSE] XYZ=({p.x:+7.3f}, {p.y:+7.3f}, {p.z:+6.3f}) m | '
                      f'ROLL={roll:+7.2f}° PITCH={pitch:+7.2f}° YAW={yaw:+7.2f}° | '
                      f'v={v.linear.x:+.2f} m/s | wz={v.angular.z:+.2f} rad/s | '
                      f'cmd=({self.commanded_speed:+.2f},{self.commanded_yaw_rate:+.2f})', flush=True)
        if self.csv_writer is not None and fresh and now - self.last_csv_at >= 0.2:
            self.last_csv_at = now
            msg = self.odom
            pos = msg.pose.pose.position
            tm = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.csv_writer.writerow([
                f'{tm:.3f}', f'{pos.x:.5f}', f'{pos.y:.5f}', f'{pos.z:.5f}',
                *(f'{a:.4f}' for a in self.angles),
                f'{msg.twist.twist.linear.x:.5f}', f'{msg.twist.twist.angular.z:.5f}',
                f'{self.commanded_speed:.5f}', f'{self.commanded_yaw_rate:.5f}'])
            self.csv_file.flush()

    def close(self):
        self.zero()
        if self.cmd_pub is not None:
            for _ in range(8):
                self.publish_cmd(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.04)
        if self.csv_file is not None:
            self.csv_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--monitor-only', action='store_true', help='Print/publish pose without controlling motors')
    parser.add_argument('--csv', default='~/ackermann_stage3_pose.csv', help='CSV telemetry path; empty disables logging')
    parser.add_argument('--deadman', type=float, default=3.0, help='Key inactivity auto-stop, seconds')
    parser.add_argument('--stale', type=float, default=2.0, help='Odometry timeout, seconds')
    parser.add_argument('--reverse-speed', type=float, default=0.10, help='Reverse speed magnitude (m/s), 0.05..0.50')
    args = parser.parse_args()
    if args.deadman <= 0 or args.stale <= 0:
        parser.error('deadman and stale timeouts must be positive')
    if not (0.05 <= args.reverse_speed <= 0.50):
        parser.error('--reverse-speed must be between 0.05 and 0.50 m/s')
    if not args.monitor_only and not sys.stdin.isatty():
        parser.error('keyboard mode needs an interactive terminal; use --monitor-only instead')
    rclpy.init()
    node = None
    old_terminal = None
    try:
        node = Stage3Console(args)
        if not args.monitor_only:
            old_terminal = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        next_publish = time.monotonic()
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.025)
            if not args.monitor_only and select.select([sys.stdin], [], [], 0)[0]:
                for ch in os.read(sys.stdin.fileno(), 256).decode('utf-8', 'ignore').lower():
                    node.key(ch)
            now = time.monotonic()
            if now >= next_publish:
                node.tick()
                next_publish = now + 0.1
    except KeyboardInterrupt:
        pass
    finally:
        if old_terminal is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_terminal)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()