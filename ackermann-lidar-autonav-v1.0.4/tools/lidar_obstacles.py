#!/usr/bin/env python3
"""Lightweight LaserScan geometry and dynamic obstacle extraction (no ROS dependency).

Scan frame: lidar_link, fixed 0.20m ahead of chassis origin; chassis is 0.40m
ahead of rear axle P. Thus LiDAR is 0.60m ahead of P along yaw.
A 2D horizontal LiDAR cannot establish obstacle height or classify terrain.
"""
from __future__ import annotations

import math
import time
from planner_core import terrain

def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def finite_return(value, minimum, maximum):
    return math.isfinite(value) and minimum <= value < maximum - 1e-3

def is_terrain_return(hit_z, ground_z, tolerance):
    """Reject a return at/below the predicted map surface.

    A collision heightmap and odom-projected scan endpoints can disagree by
    >0.20m on ramps. A point BELOW the known terrain is not evidence of an
    above-ground obstacle. A positive margin still hides very low objects;
    a 2D scan cannot distinguish all ground contacts reliably.
    """
    return (math.isfinite(hit_z) and math.isfinite(ground_z) and
            hit_z <= ground_z + max(0.0, float(tolerance)))


def is_self_return(x, y, body_x, body_y, yaw,
                   x_back=-0.64, x_front=0.67, half_width=0.43):
    """Mask only the measured robot's occupied XY footprint.

    Local X is chassis-forward and local Y is left. The bounds enclose the
    1.2x0.6 m chassis and its wheels with a small cushion, but NOT the
    whole 1.2m inflation disk used for A* safety.
    """
    dx=x-body_x
    dy=y-body_y
    forward=math.cos(yaw)*dx+math.sin(yaw)*dy
    lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
    return x_back <= forward <= x_front and abs(lateral) <= half_width


def first_terrain_intersection(ox, oy, oz, vx, vy, vz, max_distance,
                               z_shift=0.0, step=0.07, tolerance=0.03):
    """Approximate the first range at which a 3D laser beam reaches terrain.

    Rays are transformed using full roll/pitch/yaw; terrain is the known
    MATLAB heightmap. Returns inf when no terrain crossing occurs.
    Only use for a near-surface candidate, not as a global obstacle eraser.
    """
    if max_distance <= 0 or step <= 0:
        return math.inf
    previous=0.0
    count=max(1,math.ceil(max_distance/step))
    for i in range(1,count+1):
        d=min(max_distance, i*step)
        x=ox+d*vx
        y=oy+d*vy
        z=oz+d*vz
        ground=terrain(x,y)[0]+z_shift
        if z <= ground+tolerance:
            lo=previous
            hi=d
            for _ in range(9):
                mid=(lo+hi)*0.5
                hmid=terrain(ox+mid*vx,oy+mid*vy)[0]+z_shift
                if oz+mid*vz <= hmid+tolerance:
                    hi=mid
                else:
                    lo=mid
            return hi
        previous=d
    return math.inf


def rotate_quaternion(q, v):
    """Rotate local xyz by quaternion (x,y,z,w); caller supplies a unit q."""
    qx,qy,qz,qw=q
    x,y,z=v
    tx=2.0*(qy*z-qz*y)
    ty=2.0*(qz*x-qx*z)
    tz=2.0*(qx*y-qy*x)
    return (x+qw*tx+qy*tz-qz*ty,
            y+qw*ty+qz*tx-qx*tz,
            z+qw*tz+qx*ty-qy*tx)

class LidarObstacleLayer:
    def __init__(self, cfg):
        self.cfg = cfg
        self.offset_m = float(cfg.get('lidar_front_offset_from_rear_axle_m', 0.60))
        self.voxel_m = float(cfg.get('lidar_voxel_m', 0.55))
        self.max_points = int(cfg.get('lidar_max_dynamic_points', 30))
        self.ttl_s = float(cfg.get('lidar_obstacle_ttl_s', 1.0))
        self.range_cap = float(cfg.get('lidar_max_range_m', 8.0))
        self.ignore_static_extra = float(cfg.get('lidar_ignore_static_extra_m', 0.2))
        self.model_to_lidar_x = float(cfg.get('lidar_mount_x_m',0.20))
        self.model_to_lidar_z = float(cfg.get('lidar_mount_z_m',0.33))
        self.ground_tolerance = float(cfg.get('lidar_ground_tolerance_m',0.12))
        self.ground_raycast_tolerance_m = float(
            cfg.get('lidar_ground_raycast_tolerance_m',0.20))
        self.ground_raycast_height_gate_m = float(
            cfg.get('lidar_ground_raycast_height_gate_m',0.25))
        self.self_x_back = float(cfg.get('lidar_self_x_back_m',-0.64))
        self.self_x_front = float(cfg.get('lidar_self_x_front_m',0.67))
        self.self_half_width = float(cfg.get('lidar_self_half_width_m',0.43))
        self.ground_z_shift = float(cfg.get('height_shift_gazebo_m',0.0))
        self.known = [tuple(map(float, o)) for o in cfg.get('obstacles_xy_radius', [])]
        self.front_cone = math.radians(float(cfg.get('lidar_safety_cone_deg', 35.0)))
        self.last_scan_wall = None
        self._last_points = []
        self._last_direction_ranges = {1: math.inf, -1: math.inf}
        self.last_filter_stats = {
            'valid':0, 'terrain':0, 'below_map':0,
            'known_static':0, 'dynamic':0, 'self':0, 'ray_ground':0}
        if self.voxel_m <= 0 or self.max_points < 1 or self.ttl_s <= 0:
            raise ValueError('Invalid LiDAR voxel size, point count, or time-to-live')

    def update(self, ranges, angle_min, angle_increment, range_min, range_max,
               rear_x, rear_y, yaw, now=None, model_pose=None):
        """Consume one LaserScan and odometry pose, without using ground truth.

        Output dynamic endpoints in world XY, excluding known static cylinders.
        All hits (including known statics) still participate in emergency braking.
        """
        now=time.monotonic() if now is None else float(now)
        if not ranges or not math.isfinite(angle_increment) or angle_increment <= 0:
            return False
        if not (math.isfinite(angle_min) and math.isfinite(range_max)):
            return False
        maximum=min(float(range_max),self.range_cap)
        minimum=max(0.05,float(range_min))
        if minimum>=maximum:return False
        self.last_scan_wall=now
        cos=math.cos(yaw);sin=math.sin(yaw)
        ox=rear_x+self.offset_m*cos
        oy=rear_y+self.offset_m*sin
        sz=None
        q=None
        model_x=model_y=None
        model_yaw=yaw
        if model_pose is not None:
            # Odometry is model/chassis origin in world frame, including pitch
            # and roll on the ramp; ray plane is rigidly tilted with chassis.
            mx,my,mz,qx,qy,qz,qw=model_pose
            model_x,model_y=mx,my
            model_yaw=math.atan2(2*(qw*qz+qx*qy),
                                 1-2*(qy*qy+qz*qz))
            q=(qx,qy,qz,qw)
            dx,dy,dz=rotate_quaternion(q,
                (self.model_to_lidar_x,0.0,self.model_to_lidar_z))
            ox,oy,sz=mx+dx,my+dy,mz+dz
        bins={}
        stats={'valid':0,'terrain':0,'below_map':0,
               'known_static':0,'dynamic':0,'self':0,'ray_ground':0}
        direction_ranges={1:math.inf,-1:math.inf}
        for i,raw in enumerate(ranges):
            try:d=float(raw)
            except (TypeError,ValueError):continue
            if not finite_return(d,minimum,maximum):continue
            stats['valid']+=1
            a=norm_angle(angle_min+i*angle_increment)
            # Account for full chassis roll/pitch on steep terrain, otherwise
            # sloped ground can appear as a fake dynamic obstacle.
            if q is not None:
                vx,vy,vz=rotate_quaternion(q,(math.cos(a),math.sin(a),0.0))
                x=ox+d*vx;y=oy+d*vy
                if is_self_return(x,y,model_x,model_y,model_yaw,
                                  self.self_x_back,self.self_x_front,
                                  self.self_half_width):
                    stats['self']+=1
                    continue
                hit_z=sz+d*vz
                ground=terrain(x,y)[0]+self.ground_z_shift
                # Ray hits BELOW the modeled ramp are not vertical obstacles.
                # A fixed lower cutoff (ground-0.20) previously mislabelled
                # an entire x≈5m contour as a dynamic wall in v1.0.4.
                if is_terrain_return(hit_z, ground, self.ground_tolerance):
                    stats['terrain']+=1
                    if hit_z < ground-0.20:
                        stats['below_map']+=1
                    continue
                # When the measured endpoint lies a little ABOVE the
                # sampled heightmap (e.g. Gazebo 257px heightmap and a tilted
                # chassis), verify the full beam against the 3D terrain.
                # This narrow ray-range gate avoids removing tall objects.
                if 0.0 < hit_z-ground <= self.ground_raycast_height_gate_m:
                    predicted=first_terrain_intersection(
                        ox,oy,sz,vx,vy,vz,
                        min(maximum,d+self.ground_raycast_tolerance_m+0.10),
                        self.ground_z_shift)
                    if (math.isfinite(predicted) and
                        abs(d-predicted)<=self.ground_raycast_tolerance_m):
                        stats['terrain']+=1
                        stats['ray_ground']+=1
                        continue
            else:
                x=ox+d*math.cos(yaw+a);y=oy+d*math.sin(yaw+a)
                if is_self_return(
                    x,y,rear_x+0.4*math.cos(yaw),
                    rear_y+0.4*math.sin(yaw),yaw,
                    self.self_x_back,self.self_x_front,
                    self.self_half_width):
                    stats['self']+=1
                    continue
            if abs(a)<=self.front_cone:
                direction_ranges[1]=min(direction_ranges[1],d)
            if abs(norm_angle(a-math.pi))<=self.front_cone:
                direction_ranges[-1]=min(direction_ranges[-1],d)
            # Known fixed obstacles are already modeled in map-only A*.
            # Filtering their returns avoids repeatedly rerouting for them.
            if any(math.hypot(x-cx,y-cy)<=rad+self.ignore_static_extra
                   for cx,cy,rad in self.known):
                stats['known_static']+=1
                continue
            # Only retain hits inside the known heightmap bounds.
            bx=self.cfg.get('world_bounds_xy',[-20,20,-20,20])
            if not (bx[0]+0.3<x<bx[1]-0.3 and bx[2]+0.3<y<bx[3]-0.3):
                continue
            k=(round(x/self.voxel_m),round(y/self.voxel_m))
            prev=bins.get(k)
            if prev is None or d<prev[0]:bins[k]=(d,x,y)
        self._last_points=[(x,y) for d,x,y in
                            sorted(bins.values(),key=lambda item:item[0])[:self.max_points]]
        self._last_direction_ranges=direction_ranges
        stats['dynamic']=len(self._last_points)
        self.last_filter_stats=stats
        return True

    def age(self,now=None):
        now=time.monotonic() if now is None else float(now)
        return math.inf if self.last_scan_wall is None else max(0.0,now-self.last_scan_wall)

    def ready(self,now=None):
        return self.age(now)<=self.ttl_s

    def world_points(self,now=None):
        return list(self._last_points) if self.ready(now) else []

    def nearest(self,direction=1,now=None):
        if not self.ready(now):return math.nan
        return self._last_direction_ranges[1 if direction>=0 else -1]

def stopping_distance(speed, decel, reaction_s, margin_m):
    """Conservative clearance from LiDAR origin to obstacle along travel."""
    v=max(0.0,abs(float(speed)))
    a=max(0.1,float(decel))
    return max(0.1,float(margin_m))+v*max(0.0,float(reaction_s))+v*v/(2*a)
