#!/usr/bin/env python3
"""ROS 2 Humble/Gazebo11: LiDAR-aware A* navigation, force F and Ackermann delta.

The known-map v1.0.3 force plugin remains the motor controller. LiDAR adds
safety braking and bounded dynamic-obstacle A* replanning. This is a research
simulation prototype, NOT a real-robot safety-rated obstacle avoidance system.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from autonomous_force_nav import ForceNavigator
from lidar_obstacles import LidarObstacleLayer, stopping_distance
from lidar_planner import DynamicMission


class LidarNavigator(ForceNavigator):
    def __init__(self, cfg, dry_run):
        super().__init__(cfg, dry_run)
        self.mission=DynamicMission(cfg)
        self.layer=LidarObstacleLayer(cfg)
        self.scan_sub=self.create_subscription(
            LaserScan,cfg.get('lidar_scan_topic','/scan'),self.on_scan,qos_profile_sensor_data)
        self.lidar_last_log=0.0
        self.lidar_last_replan=0.0
        self.lidar_last_scan_report=0.0
        self.lidar_holding=False
        self.get_logger().info(
            '[V1.0.4] LiDAR 360deg 91 rays @5Hz, sensor=/scan, '
            'dynamic A* + collision braking, output=[F_N, delta_rad]; '
            'no camera / no cmd_vel.')

    def on_scan(self,msg):
        if self.odom is None:return
        p=self.current_pose()
        success=self.layer.update(
            msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,
            p.x,p.y,p.yaw)
        if success and time.monotonic()-self.lidar_last_scan_report>=4.0:
            self.lidar_last_scan_report=time.monotonic()
            self.get_logger().info(
                f'[LIDAR] scan OK: beams={len(msg.ranges)}, '
                f'new map points={len(self.layer.world_points())}, '
                f'front={self.layer.nearest(1):.2f}m, '
                f'rear={self.layer.nearest(-1):.2f}m')

    def braking_command(self,pose):
        v=pose.v
        if abs(v)<0.045:return 0.0
        # Negative motor torque relative to actual travel to decelerate.
        magnitude=min(self.mission.max_F,65.0+75.0*abs(v))
        return -math.copysign(magnitude,v)

    def hold(self,pose,message,now):
        self.lidar_holding=True
        self.publish(self.braking_command(pose),0.0)
        if now-self.lidar_last_log>1.0:
            self.lidar_last_log=now
            self.get_logger().warn('[LIDAR HOLD] '+message)

    def on_tick(self):
        if self.finished:
            super().on_tick()
            return
        if self.odom is None:
            super().on_tick()
            return
        now=time.monotonic()
        pose=self.current_pose()
        if self.layer.age(now)>float(self.cfg.get('lidar_stale_timeout_s',1.1)):
            self.hold(pose,'missing/stale /scan: no driving permitted',now)
            if now-self.start_wall>float(self.cfg.get('lidar_initial_wait_s',12.0)) and (
                    self.layer.last_scan_wall is None or
                    now-self.layer.last_scan_wall>float(self.cfg.get('lidar_loss_stop_s',6.0))):
                self.finish('LIDAR UNAVAILABLE: check Gazebo ray sensor plugin and /scan')
            return

        # Update live collision geometry before any force-based local rollout.
        self.mission.update_dynamic(self.layer.world_points(now))
        if self.pending_replan:
            self.publish(self.braking_command(pose),0.0)
            self.mission.path=[]
            self.pending_replan=False
            self.last_progress=(now,pose.x,pose.y)
            self.route_time=now

        direction=1 if pose.v>=0 else -1
        if abs(pose.v)<0.08:
            direction=self.mission.drive_direction
        nearest=self.layer.nearest(direction,now)
        clearance=self.cfg.get(
            'lidar_front_min_clearance_m',0.85) if direction>0 else self.cfg.get(
            'lidar_rear_min_clearance_m',1.45)
        stop_threshold=stopping_distance(
            pose.v,float(self.cfg.get('lidar_brake_decel_m_s2',1.4)),
            float(self.cfg.get('lidar_reaction_s',0.3)),float(clearance))
        if math.isfinite(nearest) and nearest<stop_threshold:
            self.hold(pose,
                f'obstacle {nearest:.2f}m in travel direction; '
                f'brake threshold {stop_threshold:.2f}m, speed={pose.v:+.2f}m/s',
                now)
            return

        # When a NEW hit intersects the remaining route, brake to a low
        # speed before asking A* for an alternate global path.
        if self.mission.path and self.mission.path_blocked((pose.x,pose.y)):
            if (abs(pose.v)<0.16 and
                now-self.lidar_last_replan>=float(self.cfg.get('lidar_replan_interval_s',1.8))):
                self.lidar_last_replan=now
                try:
                    route=self.mission.astar((pose.x,pose.y))
                    self.route_time=now
                    self.last_progress=(now,pose.x,pose.y)
                    self.get_logger().warn(
                        '[LIDAR REPLAN] '+str([(round(x,2),round(y,2)) for x,y in route]))
                except (ValueError,RuntimeError) as e:
                    self.mission.path=[]
                    self.hold(pose,'no safe detour yet: '+str(e),now)
                    return
            else:
                self.hold(pose,'new obstacle blocks global A* route; braking to replan',now)
                return

        if not self.mission.path:
            # A* may be temporarily blocked by a moving obstacle; do not
            # permanently finish the mission. Reattempt after fresh scans.
            if now-self.lidar_last_replan<float(self.cfg.get('lidar_replan_interval_s',1.8)):
                self.hold(pose,'waiting for safe dynamic-obstacle route',now)
                return
            self.lidar_last_replan=now
            try:
                route=self.mission.astar((pose.x,pose.y))
                self.route_time=now
                self.last_progress=(now,pose.x,pose.y)
                self.get_logger().info(
                    '[LIDAR A*] '+str([(round(x,2),round(y,2)) for x,y in route]))
            except (ValueError,RuntimeError) as e:
                self.hold(pose,'no safe route: '+str(e),now)
                return

        self.lidar_holding=False
        super().on_tick()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--dry-run',action='store_true',
                   help='Read /scan and preview A*, but never publish motor commands')
    args=p.parse_args()
    config=json.loads(args.config.read_text(encoding='utf-8'))
    rclpy.init()
    node=LidarNavigator(config,args.dry_run)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node,timeout_sec=0.10)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt: force command zero')
    finally:
        for _ in range(8):
            node.publish(0.0,0.0)
            rclpy.spin_once(node,timeout_sec=0.04)
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
