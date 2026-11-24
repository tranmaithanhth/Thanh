#!/usr/bin/env python3
"""ROS2 Humble navigator, known 3D map only; commands [force_N, steer_rad]."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from planner_core import Mission, Pose

CMD='/ackermann_vehicle/force_steer'
ODOM='/ackermann_vehicle/odom'


def yaw_from_quat(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class ForceNavigator(Node):
    def __init__(self, cfg, dry_run):
        super().__init__('ackermann_force_autonav')
        self.cfg=cfg
        self.mission=Mission(cfg)
        self.dry_run=dry_run
        self.odom=None
        self.odom_wall=None
        self.last_odom_sim=None
        self.cmd_pub=self.create_publisher(Float64MultiArray,CMD,10) if not dry_run else None
        self.route_pub=self.create_publisher(RosPath,'/ackermann_vehicle/planned_path',10)
        self.odom_sub=self.create_subscription(Odometry,ODOM,self.on_odom,10)
        self.timer=self.create_timer(1./cfg['planner_control_hz'],self.on_tick)
        self.last_report=0.0
        self.start_wall=time.monotonic()
        self.stuck_since=None
        self.last_progress=None
        self.finished=False
        self.status='WAITING_FOR_ODOM'
        self.route_time=0.0
        self.warned=False
        self.last_recovery_log=0.0
        self.get_logger().info('Map-only A* + force/steering tracker; NO LiDAR/camera. Robot MUST use v1.0.3 force plugin.')
        self.get_logger().info(
            f'[PLANNER CONFIG] max_grade={self.mission.max_grade:.2f}, '
            f'extra_clearance={self.mission.route_margin:.2f}m, '
            f'vref_limit={self.mission.max_speed:.2f}m/s, '
            f'force_cap={self.mission.max_F:.0f}N')
        if self.dry_run:
            self.get_logger().info(
                '[DRY RUN] Planner preview only: NO force_steer commands are published; '
                'the robot is NOT expected to move; STALL detection disabled.')

    def on_odom(self,msg):
        self.odom=msg
        self.odom_wall=time.monotonic()
        self.last_odom_sim=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def current_pose(self):
        p=self.odom.pose.pose.position
        yaw=yaw_from_quat(self.odom.pose.pose.orientation)
        offset=float(self.cfg['rear_axle_offset_from_model_origin_m'])
        return Pose(p.x-offset*math.cos(yaw),p.y-offset*math.sin(yaw),yaw,
                    self.odom.twist.twist.linear.x)

    def publish(self,force,steer):
        if self.cmd_pub is not None:
            msg=Float64MultiArray()
            msg.data=[float(force),float(steer)]
            self.cmd_pub.publish(msg)

    def publish_route(self):
        route=RosPath()
        route.header.frame_id='world'
        route.header.stamp=self.get_clock().now().to_msg()
        for x,y in self.mission.path:
            pose=PoseStamped();pose.header=route.header
            h,_,_=__import__('planner_core').terrain(x,y)
            pose.pose.position.x=x;pose.pose.position.y=y
            pose.pose.position.z=h+self.cfg['height_shift_gazebo_m']
            pose.pose.orientation.w=1.0
            route.poses.append(pose)
        self.route_pub.publish(route)

    def finish(self,reason):
        self.finished=True
        self.status=reason
        self.publish(0.0,0.0)
        self.get_logger().info(f'STOP: {reason}')

    def on_tick(self):
        if self.finished:
            self.publish(0.0,0.0)
            return
        now=time.monotonic()
        # This node is the ONLY command publisher, or gazebo may receive competing commands.
        if self.cmd_pub is not None and now-self.start_wall>2.0 and self.count_publishers(CMD)!=1:
            self.finish('multiple force_steer publishers; close other controllers')
            return
        if now-self.start_wall>self.cfg['max_runtime_s']:
            self.finish('time limit reached')
            return
        if self.odom is None:
            if now-self.start_wall>12.0:self.finish('no odom; check v1.0.3 Gazebo plugin')
            return
        if now-self.odom_wall>1.2:
            self.finish('stale odometry / Gazebo paused or frozen')
            return
        pose=self.current_pose()
        if not self.mission.path:
            try:
                path=self.mission.astar((pose.x,pose.y))
            except (ValueError,RuntimeError) as exc:
                self.finish('A* no safe route: '+str(exc));return
            self.route_time=now
            self.last_progress=(now,pose.x,pose.y)
            self.get_logger().info(f'A* planned {len(path)} waypoints: '+str([(round(x,1),round(y,1)) for x,y in path]))
        try:
            F,delta,dist,dev,vref=self.mission.local_force_steer(pose)
        except (ValueError,RuntimeError) as exc:
            self.finish('navigator computation error: '+str(exc));return
        if dist<self.cfg['goal_tolerance_m'] and abs(pose.v)<0.06:
            self.finish(f'GOAL REACHED, XY error {dist:.2f} m')
            return
        if dist>=self.cfg['goal_tolerance_m'] and F==0.0 and delta==0.0 and vref==0.0:
            # Local planner found no safe rollout. Never force it forward
            # blindly: report cause promptly rather than silently stalling.
            self.finish('NO SAFE LOCAL ROLLOUT: robot/route near obstacle or grade boundary;'
                        ' reset Gazebo and inspect A* path and terrain safety margin')
            return
        if dev>self.cfg['replan_after_deviation_m'] and now-self.route_time>5.0:
            try:
                self.mission.astar((pose.x,pose.y))
                self.route_time=now
                self.get_logger().warn(f'Off-track {dev:.2f}m: replanned from current pose')
            except Exception as exc:
                self.finish('Off-route, A* failed: '+str(exc));return
        if not self.mission.safe_motion(pose.x,pose.y,pose.yaw):
            self.finish('Current pose unsafe for obstacle/grade/bounds');return
        # Only the live controller can be judged by physical progress. In
        # --dry-run we deliberately do NOT publish force/steer commands, so
        # stationary odometry is an expected result, never a drivetrain STALL.
        if not self.dry_run:
            if self.last_progress is None or math.hypot(pose.x-self.last_progress[1],pose.y-self.last_progress[2])>=0.2:
                self.last_progress=(now,pose.x,pose.y)
            elif now-self.last_progress[0]>self.cfg['stuck_timeout_s'] and dist>self.cfg['goal_tolerance_m']:
                self.finish(f'STALL: <0.20m progress over {self.cfg["stuck_timeout_s"]}s; inspect wheel joints, force output and traction')
                return
        # Recover traction at a ramp only when REAL XY progress is absent.
        # Prefer measured progress (not odom twist alone: the body can vibrate
        # and report v=0.1m/s while its world XYZ remains unchanged).
        # Do not increase force near obstacles or beyond the configured cap.
        if not self.dry_run and self.last_progress is not None and vref>0.25:
            rear_grade,front_grade=self.mission.axle_grades(pose)
            stagnant_s=now-self.last_progress[0]
            if (max(rear_grade,front_grade)>=self.cfg.get('recovery_min_front_grade',0.06)
                    and stagnant_s>=self.cfg.get('recovery_after_s',2.0)
                    and pose.v<0.35):
                candidate=min(
                    self.mission.max_F,
                    float(self.cfg.get('recovery_max_force_N',180.0)),
                    float(self.cfg.get('recovery_initial_force_N',75.0))+
                    float(self.cfg.get('recovery_ramp_N_per_s',22.0))*
                    (stagnant_s-float(self.cfg.get('recovery_after_s',2.0))))
                if candidate>F and self.mission.rollout_safe(
                        pose,candidate,delta,
                        float(self.cfg.get('recovery_horizon_s',0.45))):
                    F=candidate
                    if now-self.last_recovery_log>1.0:
                        self.last_recovery_log=now
                        self.get_logger().warn(
                            f'[HILL RECOVERY] stationary={stagnant_s:.1f}s, '
                            f'rear_grade={rear_grade:+.3f}, front_grade={front_grade:+.3f}, '
                            f'boost F={F:.1f}N (cap {self.mission.max_F:.0f}N)')
        self.publish(F,delta)
        self.publish_route()  # published at 10 Hz; allows RViz to join without transient QoS
        if now-self.last_report>=1.0/self.cfg['logging_hz']:
            self.last_report=now
            self.get_logger().info(f'[AUTO] P=({pose.x:+.2f},{pose.y:+.2f}) yaw={math.degrees(pose.yaw):+.1f}deg '
                                   f'v={pose.v:+.2f}m/s vref={vref:.2f} F={F:+.1f}N '
                                   f'delta={math.degrees(delta):+.1f}deg goalErr={dist:.2f}m routeErr={dev:.2f}m '
                                   f'frontGrade={self.mission.axle_grades(pose)[1]:+.3f}'
                                   +(' [DRY RUN]' if self.dry_run else ''))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--dry-run',action='store_true',help='Observe and plan, but do not apply driving forces')
    args=ap.parse_args()
    cfg=json.loads(args.config.read_text(encoding='utf-8'))
    rclpy.init()
    node=ForceNavigator(cfg,args.dry_run)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node,timeout_sec=0.10)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt: stop')
    finally:
        for _ in range(8):
            node.publish(0.0,0.0)
            rclpy.spin_once(node,timeout_sec=0.04)
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':main()