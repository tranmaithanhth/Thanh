#!/usr/bin/env python3
"""Local *simulator-truth* proximity sensor for Gazebo Classic 11.

No LaserScan, camera or preloaded obstacle XY positions are used. This is an
oracle approximation for RESEARCH SIMULATION, not a physical robot sensor.
Only object centers whose closest circular edge is within detection radius
are revealed to the route planner; previous observations may be remembered.
"""
from __future__ import annotations
import math
import time

# Infrastructure is not a newly discovered movable obstacle. The terrain
# and world boundaries are already constrained geometrically in Mission.
EXACT_IGNORED = frozenset((
    "ground_plane", "sun", "ackermann_matlab_terrain_v102",
    "start_marker", "goal_marker",
))
IGNORED_PREFIXES = ("boundary_", "ackermann_force_", "ackermann_vehicle")


def is_environment(name):
    return (name in EXACT_IGNORED or
            any(name.startswith(prefix) for prefix in IGNORED_PREFIXES))


def stopping_distance(speed, decel, reaction_s, margin_m):
    """Conservative commanded-motor braking envelope; not a parking brake."""
    v=abs(float(speed))
    a=max(0.1,float(decel))
    return max(0.1,float(margin_m))+v*max(0.0,float(reaction_s))+v*v/(2.0*a)


def closest_edge_distance(x, y, radius, px, py):
    return max(0., math.hypot(x-px, y-py)-max(0., radius))


def sensed_models(model_names, poses, px, py, sensing_radius_m,
                  default_obstacle_radius_m=0.75):
    """Return (name,x,y,r) for objects within a bounded local 360° disk.

    No model position is considered outside the sensed radius. The pose array
    comes from Gazebo state, NOT from the map/mission's known obstacle list.
    Approximate circular geometry is deliberately conservative: ModelStates
    reports pose but not exact collision dimensions.
    """
    if sensing_radius_m <= 0 or default_obstacle_radius_m <= 0:
        raise ValueError("Invalid virtual proximity sensor range or object radius")
    nearby=[]
    for name,pose in zip(model_names,poses):
        if is_environment(name): continue
        x=float(pose.position.x); y=float(pose.position.y)
        if not all(math.isfinite(v) for v in (x,y)):continue
        r=float(default_obstacle_radius_m)
        if closest_edge_distance(x,y,r,px,py)<=sensing_radius_m:
            nearby.append((str(name),x,y,r))
    return nearby


def nearest_clearance(obstacles, px, py, yaw, direction, front_extent_m=0.8,
                      rear_extent_m=0.22, half_width_m=0.4):
    """Free distance from the vehicle bumper in its forward/reverse corridor.

    The obstacle must overlap the swept lateral corridor; a distant object
    off to the side never triggers forward-only braking.
    """
    best=math.inf
    direction=1 if direction>=0 else -1
    c=math.cos(yaw);s=math.sin(yaw)
    for item in obstacles:
        _,x,y,r = item
        dx=x-px;dy=y-py
        longitudinal=direction*(c*dx+s*dy)
        lateral=abs(-s*dx+c*dy)
        if lateral>r+half_width_m:continue
        if longitudinal < -r:continue
        bumper=front_extent_m if direction>0 else rear_extent_m
        best=min(best,max(0.0,longitudinal-r-bumper))
    return best


class NearbyScene:
    def __init__(self, radius_m=4.0, default_radius_m=0.75,
                 memory_s=60.0):
        if radius_m <= 0 or default_radius_m <= 0 or memory_s <= 0:
            raise ValueError("Invalid NearbyScene parameters")
        self.radius_m=float(radius_m)
        self.default_radius_m=float(default_radius_m)
        self.memory_s=float(memory_s)
        self.last_update_wall=None
        self.visible=[]
        self._learned={}

    def ingest(self, names, poses, px, py, now=None):
        now=time.monotonic() if now is None else float(now)
        current=sensed_models(names,poses,px,py,self.radius_m,
                              self.default_radius_m)
        self.visible=current
        for item in current:
            self._learned[item[0]]=(now,item)
        self.last_update_wall=now
        self._forget_old(now)
        return list(current)

    def _forget_old(self,now):
        for name,(stamp,_) in list(self._learned.items()):
            if now-stamp>self.memory_s:
                del self._learned[name]

    def age(self,now=None):
        now=time.monotonic() if now is None else float(now)
        return (math.inf if self.last_update_wall is None else
                max(0.,now-self.last_update_wall))

    def remembered(self,now=None):
        now=time.monotonic() if now is None else float(now)
        self._forget_old(now)
        return [item for _,item in self._learned.values()]

    def packet(self):
        """ROS Float64MultiArray payload: x,y,r repeated for visible items."""
        return [v for _,x,y,r in self.visible for v in (x,y,r)]

    @staticmethod
    def unpack_packet(data):
        if len(data)%3:
            raise ValueError("nearby obstacles message must contain x,y,r triplets")
        values=[float(x) for x in data]
        if any(not math.isfinite(v) for v in values) or any(
                values[i+2]<=0 for i in range(0,len(values),3)):
            raise ValueError("nearby packet contains invalid obstacle")
        return [tuple(values[i:i+3]) for i in range(0,len(values),3)]
