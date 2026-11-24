#!/usr/bin/env python3
"""Dynamic obstacle overlay for the v1.0.3 force/steering Ackermann A* planner."""
from __future__ import annotations
import math
from planner_core import Mission,clamp


class DynamicMission(Mission):
    def __init__(self, cfg):
        self.dynamic_obstacles=[]
        self.dynamic_radius=float(cfg.get('lidar_dynamic_radius_m',0.28))
        super().__init__(cfg)

    def update_dynamic(self, world_xy):
        """Update LiDAR obstacle points. Rebuild occupancy lazily during A*."""
        new=[]
        for x,y in world_xy:
            if math.isfinite(x) and math.isfinite(y):
                new.append((float(x),float(y),self.dynamic_radius))
        self.dynamic_obstacles=new[:int(self.c.get('lidar_max_dynamic_points',30))]

    def safe_xy(self,x,y,margin=0.0):
        if not super().safe_xy(x,y,margin=margin):return False
        # Include footprint inflation in addition to each detected obstacle.
        return all(math.hypot(x-ox,y-oy)>r+self.infl+margin
                   for ox,oy,r in self.dynamic_obstacles)

    def astar(self,start):
        # Global grid occupancy depends on dynamic point clouds; rebuild only
        # at replanning, not on every 10Hz control iteration.
        self.occupied=[[not self.safe_xy(*self.to_world(i,j),
                                           margin=self.route_margin)
                        for i in range(self.nx)] for j in range(self.ny)]
        return super().astar(start)

    def path_blocked(self, current_xy):
        """Is a fresh LiDAR return near the A* route *ahead* of the robot?"""
        if len(self.path)<2 or not self.dynamic_obstacles:return False
        current_s,_,_,_=self.nearest_s_and_target(current_xy)
        cum=0.0
        for a,b in zip(self.path,self.path[1:]):
            dx,dy=b[0]-a[0],b[1]-a[1]
            length=math.hypot(dx,dy)
            if length<1e-8:continue
            for ox,oy,r in self.dynamic_obstacles:
                u=clamp(((ox-a[0])*dx+(oy-a[1])*dy)/(length*length),0.0,1.0)
                s=cum+length*u
                sep=math.hypot(ox-(a[0]+u*dx),oy-(a[1]+u*dy))
                if s>=current_s-0.25 and sep<r+self.infl+self.route_margin:
                    return True
            cum+=length
        return False
