#!/usr/bin/env python3
"""Slow straight-line slope traverse demo for ackermann_vehicle on terrain.

Starts from the configured spawn pose (x=0,y=-2,yaw=0), drives ~15 m along
+X at <=0.45m/s.  This is a dynamics / Gazebo contact demo, NOT a DWA/TEB
planner and it does not attempt to reach the green marker at (16,0.6).
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TOPIC = '/ackermann_vehicle/cmd_vel'
ODOM_TOPIC = '/ackermann_vehicle/odom'
TARGET_M = 15.0
MAX_SPEED_MPS = 0.45

class SlopeDrive(Node):
    def __init__(self):
        super().__init__('ackermann_terrain_slope_demo')
        self.cmd_pub = self.create_publisher(Twist, TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.start_pose = None
        self.last_pose = None
        self.last_odom_wall = None
        self.started = time.monotonic()
        self.moved_start = None
        self.finished = False
        self.reason = ''
        self.distance = 0.0
        self.last_report = 0.0
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info('Waiting for ackermann odometry. Launch Gazebo and spawn robot first.')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        pos = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(a) for a in pos):
            return
        if self.start_pose is None:
            self.start_pose = pos
            self.get_logger().info('Initial odom position: (%.2f, %.2f, %.2f)' % pos)
        self.last_pose = pos
        self.last_odom_wall = time.monotonic()
        self.distance = math.hypot(pos[0] - self.start_pose[0], pos[1] - self.start_pose[1])

    def stop(self, reason):
        self.reason = reason
        self.finished = True
        self.get_logger().info('STOP: '+reason)
        self.publish(0.0)

    def publish(self, speed):
        t = Twist()
        t.linear.x = float(speed)
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def step(self):
        now = time.monotonic()
        if self.finished:
            self.publish(0.0)
            return
        if now - self.started > 95.0:
            self.stop('time limit: 95 seconds')
            return
        if self.last_odom_wall is None:
            self.publish(0.0)
            if now - self.started > 12.0:
                self.stop('no /ackermann_vehicle/odom received; check plugin & topics')
            return
        if now - self.last_odom_wall > 2.0:
            self.stop('odometry stale; stopping to avoid uncontrolled motion')
            return
        if self.distance >= TARGET_M:
            self.stop('travelled %.2f m across terrain' % self.distance)
            return
        if self.moved_start is None:
            self.moved_start = now
        if now - self.moved_start > 14.0 and self.distance < 0.15:
            self.stop('commands sent but robot did not move; check Gazebo drive plugin')
            return
        # Approximate acceleration/deceleration limits; the Gazebo plugin
        # remains responsible for the actual wheel dynamics.
        ramp_speed = min(MAX_SPEED_MPS, 0.12 + 0.05 * self.distance)
        remaining = TARGET_M - self.distance
        slow_speed = max(0.12, min(MAX_SPEED_MPS, 0.20 * remaining))
        speed = min(ramp_speed,slow_speed)
        self.publish(speed)
        if now - self.last_report >= 3.0:
            self.last_report = now
            self.get_logger().info('Distance %.2f / %.2f m | speed ref %.2f m/s | height z %.2f m' %
                                   (self.distance, TARGET_M, speed, self.last_pose[2]))


def main():
    rclpy.init()
    node = SlopeDrive()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node,timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        # Multiple zero messages give plugin time to receive a stop command.
        for _ in range(7):
            node.publish(0.0)
            rclpy.spin_once(node,timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()