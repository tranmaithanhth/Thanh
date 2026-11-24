#!/usr/bin/env python3
"""Gazebo truth NEARBY virtual sensor + incremental Ackermann A* (NO LiDAR).

ModelStates lists all Gazebo objects; the ONLY planner-visible observation is
the bounded local subset computed in NearbyScene. Route starts with zero known
obstacles; the source terrain and actual Gazebo collision objects are unchanged.
Not a real-robot perception substitute or safety-rated collision system.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped

from autonomous_force_nav import ForceNavigator
from local_planners_v104 import LocalPlanningMission, MODES
from planner_core import terrain
from nearby_scene import NearbyScene, nearest_clearance, stopping_distance

SCENE_TOPIC='/gazebo/model_states'
LOCAL_TOPIC='/ackermann_vehicle/nearby_obstacles'
CMD='/ackermann_vehicle/force_steer'
LOCAL_PATH='/ackermann_vehicle/local_path'
LOCAL_STATUS='/ackermann_vehicle/local_planner_status'


class NearbyNavigator(ForceNavigator):
    def __init__(self, cfg, dry_run):
        # Important: never load physical obstacles as a priori A* data.
        cfg=dict(cfg)
        cfg['obstacles_xy_radius']=[]
        super().__init__(cfg, dry_run)
        self.mission=LocalPlanningMission(cfg)
        self.scene=NearbyScene(
            radius_m=cfg.get('nearby_radius_m',4.0),
            default_radius_m=cfg.get('nearby_obstacle_radius_m',0.75),
            memory_s=cfg.get('nearby_memory_s',60.0))
        self.scene_sub=self.create_subscription(
            ModelStates,SCENE_TOPIC,self.on_scene,10)
        self.nearby_pub=self.create_publisher(Float64MultiArray,LOCAL_TOPIC,10)
        self.local_path_pub=self.create_publisher(RosPath,LOCAL_PATH,10)
        self.local_status_pub=self.create_publisher(String,LOCAL_STATUS,10)
        self.local_last_log=0.0
        self.local_last_fallback=None
        self.last_scene_log=0.0
        self.last_brake_log=0.0
        self.last_replan=0.0
        self.last_seen_names=set()
        self.last_visible=[]
        self.local_recovery_since=None
        self.local_recovery_last_log=0.0
        self.local_recovery_attempts=0
        self.get_logger().info(
            '[NEARBY] simulator-only bounded neighborhood radius=%.1fm; '
            'no /scan subscription; no preloaded obstacle XY coordinates; '
            'A* global + %s local planner + [F,delta] command' %
            (self.scene.radius_m,self.mission.mode))

    def on_scene(self,msg):
        if self.odom is None:
            return
        pose=self.current_pose()
        observed=self.scene.ingest(msg.name,msg.pose,pose.x,pose.y)
        self.last_visible=observed
        packet=Float64MultiArray()
        packet.data=self.scene.packet()
        # Empty packet is also a heartbeat: no nearby obstacles.
        self.nearby_pub.publish(packet)
        now=time.monotonic()
        if now-self.last_scene_log>=2.0:
            self.last_scene_log=now
            self.get_logger().info(
                '[NEARBY] visible=%d remembered=%d radius=%.1fm '
                'sample=%s' % (
                    len(observed),len(self.scene.remembered(now)),
                    self.scene.radius_m,
                    [(name,round(x,2),round(y,2)) for name,x,y,r
                     in observed[:5]]))

    def publish_local_path(self):
        msg=RosPath()
        msg.header.frame_id='world'
        msg.header.stamp=self.get_clock().now().to_msg()
        for x,y in self.mission.local_path:
            pose=PoseStamped()
            pose.header=msg.header
            pose.pose.position.x=float(x)
            pose.pose.position.y=float(y)
            pose.pose.position.z=terrain(x,y)[0]+self.cfg['height_shift_gazebo_m']
            pose.pose.orientation.w=1.0
            msg.poses.append(pose)
        self.local_path_pub.publish(msg)

    def finish(self,reason):
        """Intercept the base tracker terminal NO SAFE response for DWA/TEB.

        It used to stop the whole run immediately. With new nearby objects,
        the safe reaction is to brake and retry a fresh global route at low
        speed, subject to a bounded recovery deadline. Never enable a blind
        forward command or weaken obstacle inflation.
        """
        if (reason.startswith('NO SAFE LOCAL ROLLOUT') and
                self.mission.mode in ('DWA','TEB') and self.odom is not None):
            now=time.monotonic()
            pose=self.current_pose()
            if self.local_recovery_since is None:
                self.local_recovery_since=now
                self.local_recovery_attempts=0
            elapsed=now-self.local_recovery_since
            if elapsed>float(self.cfg.get('local_recovery_timeout_s',12.)):
                super().finish(
                    'NO SAFE DWA/TEB after %.1fs of braking/replanning; '
                    'pose=(%.2f,%.2f) yaw=%.1fdeg obstacle inflation kept' %
                    (elapsed,pose.x,pose.y,math.degrees(pose.yaw)))
                return
            self.hold(pose,'DWA/TEB: no safe 2.5s candidate; braking',now)
            if now-self.local_recovery_last_log>1.0:
                self.local_recovery_last_log=now
                self.get_logger().warn(
                    '[LOCAL RECOVERY] solver=%s valid=%d/%d v=%+.2f '
                    'time=%.1fs attempts=%d' % (
                    self.mission.last_solver,
                    self.mission.last_dwa_valid_candidates,
                    self.mission.last_dwa_tested_candidates,
                    pose.v,elapsed,self.local_recovery_attempts))
            if (abs(pose.v)<.12 and
                now-self.last_replan>=float(
                    self.cfg.get('nearby_replan_interval_s',1.8))):
                self.local_recovery_attempts+=1
                self._try_replan(pose,now,'[LOCAL RECOVERY A*]')
                # Base class already tries a safe reverse candidate when
                # stopped. Do not force a blind reverse command here.
            return
        super().finish(reason)

    def braking_command(self,pose):
        if abs(pose.v)<0.045:return 0.0
        magnitude=min(self.mission.max_F,65.0+75.0*abs(pose.v))
        return -math.copysign(magnitude,pose.v)

    def hold(self,pose,reason,now):
        self.mission.local_path=[]
        self.publish_local_path()
        self.publish(self.braking_command(pose),0.0)
        if now-self.last_brake_log>1.0:
            self.last_brake_log=now
            self.get_logger().warn('[NEARBY HOLD] '+reason)

    def _try_replan(self,pose,now,prefix):
        self.last_replan=now
        try:
            route=self.mission.astar((pose.x,pose.y))
            self.route_time=now
            self.last_progress=(now,pose.x,pose.y)
            self.publish_route()
            self.get_logger().warn(
                prefix+' '+str([(round(x,2),round(y,2)) for x,y in route]))
            return True
        except (ValueError,RuntimeError) as exc:
            self.mission.path=[]
            self.mission.local_path=[]
            self.publish_local_path()
            self.hold(pose,'no safe local-map route: '+str(exc),now)
            return False

    def on_tick(self):
        if self.finished or self.odom is None:
            super().on_tick()
            return
        now=time.monotonic()
        if now-self.start_wall>self.cfg['max_runtime_s']:
            self.finish('time limit reached')
            return
        if self.odom_wall is None or now-self.odom_wall>1.2:
            self.finish('stale odometry / Gazebo paused or frozen')
            return
        if (self.cmd_pub is not None and now-self.start_wall>2.0 and
                self.count_publishers(CMD)!=1):
            self.finish('multiple force_steer publishers; close other controllers')
            return
        pose=self.current_pose()
        if self.scene.age(now)>self.cfg.get('nearby_stale_timeout_s',1.3):
            self.hold(pose,'missing/stale /gazebo/model_states; zero drive',now)
            if now-self.start_wall>self.cfg.get('nearby_initial_wait_s',12.0) and (
                    self.scene.last_update_wall is None or
                    self.scene.age(now)>self.cfg.get('nearby_loss_stop_s',6.0)):
                self.finish('NEARBY STATE UNAVAILABLE: start Gazebo using run_world_v104.sh')
            return

        # The planner receives ONLY previously detected nearby objects, not
        # positions from mission.json or distant world models.
        self.mission.update_dynamic(self.scene.remembered(now))
        # Earlier than the legacy 2m deviation trigger, replan if the
        # vehicle is visibly leaving the route while near a discovered
        # object. The reported failure climbed from 0.73 to 1.03m before
        # the short rollout became infeasible.
        if (len(self.mission.path)>=2 and self.last_visible and
            now-self.route_time>2.0 and
            now-self.last_replan>=float(
                self.cfg.get('nearby_replan_interval_s',1.8))):
            _,dev,_,_=self.mission.nearest_s_and_target((pose.x,pose.y))
            if (dev>float(self.cfg.get('nearby_early_replan_dev_m',0.95))
                and any(math.hypot(pose.x-x,pose.y-y)<r+3.0
                        for _,x,y,r in self.last_visible)):
                if not self._try_replan(pose,now,'[NEARBY DEVIATION REPLAN]'):
                    return
        if self.pending_replan:
            self.publish(self.braking_command(pose),0.0)
            self.mission.path=[]
            self.pending_replan=False
            self.last_progress=(now,pose.x,pose.y)
            self.route_time=now

        direction=1 if pose.v>=0 else -1
        if abs(pose.v)<0.08:
            direction=self.mission.drive_direction
        nearest=nearest_clearance(
            self.last_visible,pose.x,pose.y,pose.yaw,direction,
            front_extent_m=self.cfg.get('nearby_front_extent_m',0.8),
            rear_extent_m=self.cfg.get('nearby_rear_extent_m',0.22),
            half_width_m=self.cfg.get('nearby_half_width_m',0.4))
        clearance=self.cfg.get(
            'nearby_front_min_clearance_m',0.55) if direction>0 else self.cfg.get(
            'nearby_rear_min_clearance_m',0.65)
        threshold=stopping_distance(
            pose.v,self.cfg.get('nearby_brake_decel_m_s2',1.4),
            self.cfg.get('nearby_reaction_s',0.3),clearance)

        if self.mission.path and self.mission.path_blocked((pose.x,pose.y)):
            if (abs(pose.v)<0.16 and
                now-self.last_replan>=self.cfg.get('nearby_replan_interval_s',1.8)):
                if not self._try_replan(pose,now,'[NEARBY REPLAN]'):
                    return
            else:
                self.hold(pose,'observed obstacle blocks route; braking to replan',now)
                return

        # Even when an obstacle is close, attempt a route update while
        # stationary. Never move into the object to force a path.
        if not self.mission.path:
            if now-self.last_replan<self.cfg.get('nearby_replan_interval_s',1.8):
                self.hold(pose,'waiting for safe route',now)
                return
            if not self._try_replan(pose,now,'[NEARBY A*]'):
                return

        if math.isfinite(nearest) and nearest<threshold:
            self.hold(pose,
                'object %.2fm from bumper; threshold %.2fm; v=%+.2fm/s' %
                (nearest,threshold,pose.v),now)
            return

        super().on_tick()
        # Actual local trajectory selected by DWA/TEB; never confuse with A*.
        self.publish_local_path()
        msg=String()
        msg.data=json.dumps({
            'requested':self.mission.mode,
            'effective':self.mission.last_solver,
            'dwa_valid':self.mission.last_dwa_valid_candidates,
            'dwa_tested':self.mission.last_dwa_tested_candidates,
            'fallback':self.mission.fallback_active,
            'no_safe':self.mission.no_safe_rollout,
        })
        self.local_status_pub.publish(msg)
        if (not self.mission.no_safe_rollout and not self.finished and
                self.local_recovery_since is not None):
            self.get_logger().info(
                '[LOCAL RECOVERY] feasible motion found; resume planner')
            self.local_recovery_since=None
        if now-self.local_last_log>=1.0:
            self.local_last_log=now
            self.get_logger().info(
                '[LOCAL] requested=%s effective=%s '
                'DWA_valid=%d/%d predicted_points=%d cost=%.3f '
                'fallback=%s vref=%+.2f no_safe=%s' % (
                    self.mission.mode,self.mission.last_solver,
                    self.mission.last_dwa_valid_candidates,
                    self.mission.last_dwa_tested_candidates,
                    len(self.mission.local_path),self.mission.local_cost,
                    self.mission.fallback_active,
                    self.mission.last_reference_v,self.mission.no_safe_rollout))
            self.local_last_fallback=self.mission.fallback_active


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--dry-run',action='store_true',
                    help='observe and plan only; publish NO motor commands')
    ap.add_argument('--local-planner',choices=MODES,
                    help='DWA, TEB or previous collision-checked ROLLOUT')
    args=ap.parse_args()
    cfg=json.loads(args.config.read_text(encoding='utf-8'))
    if args.local_planner:
        cfg['local_planner']=args.local_planner
    rclpy.init()
    node=NearbyNavigator(cfg,args.dry_run)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node,timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(8):
            node.publish(0.0,0.0)
            rclpy.spin_once(node,timeout_sec=0.03)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
