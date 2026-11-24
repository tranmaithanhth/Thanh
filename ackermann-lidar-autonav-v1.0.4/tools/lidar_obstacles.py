#!/usr/bin/env python3
"""Lightweight LaserScan geometry and dynamic obstacle extraction (no ROS dependency).

Scan frame: lidar_link, fixed 0.20m ahead of chassis origin; chassis is 0.40m
ahead of rear axle P. Thus LiDAR is 0.60m ahead of P along yaw.
A 2D horizontal LiDAR cannot establish obstacle height or classify terrain.
"""
from __future__ import annotations

import math
import time

def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def finite_return(value, minimum, maximum):
    return math.isfinite(value) and minimum <= value < maximum - 1e-3

class LidarObstacleLayer:
    def __init__(self, cfg):
        self.cfg = cfg
        self.offset_m = float(cfg.get('lidar_front_offset_from_rear_axle_m', 0.60))
        self.voxel_m = float(cfg.get('lidar_voxel_m', 0.55))
        self.max_points = int(cfg.get('lidar_max_dynamic_points', 30))
        self.ttl_s = float(cfg.get('lidar_obstacle_ttl_s', 1.0))
        self.range_cap = float(cfg.get('lidar_max_range_m', 8.0))
        self.ignore_static_extra = float(cfg.get('lidar_ignore_static_extra_m', 0.2))
        self.known = [tuple(map(float, o)) for o in cfg.get('obstacles_xy_radius', [])]
        self.front_cone = math.radians(float(cfg.get('lidar_safety_cone_deg', 35.0)))
        self.last_scan_wall = None
        self._last_points = []
        self._last_direction_ranges = {1: math.inf, -1: math.inf}
        if self.voxel_m <= 0 or self.max_points < 1 or self.ttl_s <= 0:
            raise ValueError('Invalid LiDAR voxel size, point count, or time-to-live')

    def update(self, ranges, angle_min, angle_increment, range_min, range_max,
               rear_x, rear_y, yaw, now=None):
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
        bins={}
        direction_ranges={1:math.inf,-1:math.inf}
        for i,raw in enumerate(ranges):
            try:d=float(raw)
            except (TypeError,ValueError):continue
            if not finite_return(d,minimum,maximum):continue
            a=norm_angle(angle_min+i*angle_increment)
            if abs(a)<=self.front_cone:
                direction_ranges[1]=min(direction_ranges[1],d)
            if abs(norm_angle(a-math.pi))<=self.front_cone:
                direction_ranges[-1]=min(direction_ranges[-1],d)
            # Account for current chassis heading; sensor rays are yaw-relative.
            x=ox+d*math.cos(yaw+a);y=oy+d*math.sin(yaw+a)
            # Known fixed obstacles are already modeled in map-only A*.
            # Filtering their returns avoids repeatedly rerouting for them.
            if any(math.hypot(x-cx,y-cy)<=rad+self.ignore_static_extra
                   for cx,cy,rad in self.known):
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
