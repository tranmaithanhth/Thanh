#!/usr/bin/env python3
"""Pure Python: 3D terrain -> A* -> collision-aware force + Ackermann steering.

No LiDAR, camera, ROS dependency or Gazebo plugin needed to unit-test.
World known from original MATLAB Terrain3D v1.0.4 static obstacle map.
This is a geometric A* + force-based rollout tracker, NOT a full MATLAB DWA/TEB port.
"""
from __future__ import annotations
import heapq
import math
from dataclasses import dataclass
from typing import List, Tuple


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_pi(x):
    return math.atan2(math.sin(x), math.cos(x))


def smootherstep(x, a, b):
    t = clamp((x - a) / (b - a), 0., 1.)
    s = 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t
    d = (30*t*t - 60*t*t*t + 30*t*t*t*t)/(b-a) if a < x < b else 0.
    return s, d


def terrain(x, y):
    """Returns height h and gradients hx,hy of MATLAB Terrain3D v1.0.4."""
    a, da = smootherstep(x, 2.5, 6.5)
    b, db = smootherstep(x, 10.6, 14.6)
    th = math.tanh(y / 2.5)
    wave = 0.45*x + 0.5*y
    h = 1.05*(a-b)+0.045*math.sin(0.55*x)*th+0.018*math.sin(wave)
    hx = 1.05*(da-db)+0.55*0.045*math.cos(0.55*x)*th+0.45*0.018*math.cos(wave)
    hy = 0.045/2.5*math.sin(0.55*x)*(1-th*th)+0.5*0.018*math.cos(wave)
    return h, hx, hy


@dataclass
class Pose:
    x: float  # rear axle midpoint P, not Gazebo body center
    y: float
    yaw: float
    v: float


class Mission:
    def __init__(self, config):
        self.c = config
        self.obstacles = [tuple(map(float, q)) for q in config['obstacles_xy_radius']]
        self.infl = float(config['robot_inflation_radius_m'])
        self.bounds = tuple(map(float, config['route_bounds_xy']))
        self.grid = float(config['global_grid_resolution_m'])
        self.max_grade = float(config['max_terrain_grade'])
        self.cross_grade = float(config['max_cross_grade'])
        self.goal = tuple(map(float, config['goal_xy']))
        self.L = float(config['wheelbase_m'])
        self.Rw = float(config['wheel_radius_m'])
        self.max_F = float(config['max_drive_force_N'])
        self.hill_start_force = float(config.get('hill_start_force_N', 0.0))
        self.hill_start_min_grade = float(config.get('hill_start_min_grade', 0.16))
        if not (0.0 <= self.hill_start_force <= self.max_F):
            raise ValueError('hill_start_force_N must lie between 0 and max_drive_force_N')
        self.max_delta = float(config['max_steer_rad'])
        self.mass = float(config['vehicle_mass_kg'])
        self.eq_mass = self.mass + float(config['wheel_rotational_inertia_total_kgm2'])/self.Rw**2
        self.c_roll = float(config['rolling_viscous_N_s_m'])
        self.max_speed = float(config['max_speed_m_s'])
        self.goal_tolerance = float(config['goal_tolerance_m'])
        self.lookahead = float(config['lookahead_m'])
        if self.grid <= 0 or self.max_F <= 0 or self.L <= 0 or self.goal_tolerance <= 0:
            raise ValueError('Invalid mission model parameters')
        self.nx = round((self.bounds[1]-self.bounds[0])/self.grid)+1
        self.ny = round((self.bounds[3]-self.bounds[2])/self.grid)+1
        self.occupied = [[not self.safe_xy(self.to_world(i,j)[0],self.to_world(i,j)[1])
                          for i in range(self.nx)] for j in range(self.ny)]
        self.path = []

    def to_grid(self, x, y):
        return (round((x-self.bounds[0])/self.grid), round((y-self.bounds[2])/self.grid))

    def to_world(self, i, j):
        return (self.bounds[0]+i*self.grid, self.bounds[2]+j*self.grid)

    def safe_xy(self,x,y):
        if x <= self.bounds[0]+0.35 or x >= self.bounds[1]-0.35 or y <= self.bounds[2]+0.35 or y >= self.bounds[3]-0.35:
            return False
        _, hx, hy = terrain(x,y)
        if math.hypot(hx,hy)>self.max_grade:
            return False
        return all(math.hypot(x-ox,y-oy) > rad+self.infl for ox,oy,rad in self.obstacles)

    def safe_motion(self,x,y,yaw):
        if not self.safe_xy(x,y): return False
        _,hx,hy=terrain(x,y)
        grade=hx*math.cos(yaw)+hy*math.sin(yaw)
        cross=-hx*math.sin(yaw)+hy*math.cos(yaw)
        return abs(grade)<=self.max_grade and abs(cross)<=self.cross_grade

    def segment_safe(self,a,b):
        distance = math.hypot(b[0]-a[0], b[1]-a[1])
        steps = max(1, math.ceil(distance/0.10))
        for k in range(steps+1):
            t=k/steps
            if not self.safe_xy(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])):return False
        return True

    def astar(self,start):
        if not self.safe_xy(*start) or not self.safe_xy(*self.goal):
            raise ValueError(f'Start or goal unsafe: start={start} goal={self.goal}; change mission bounds/spawn')
        if self.segment_safe(start,self.goal):
            self.path=[tuple(start),self.goal]
            return self.path
        si,sj=self.to_grid(*start)
        gi,gj=self.to_grid(*self.goal)
        def passable(i,j):
            return 0<=i<self.nx and 0<=j<self.ny and not self.occupied[j][i]
        if not passable(si,sj) or not passable(gi,gj):
            raise ValueError('Grid resolution snapped start/goal into obstacle; reduce resolution')
        start_id=(si,sj); goal_id=(gi,gj)
        costs={start_id:0.0}; parents={}; open_set=[]; counter=0
        heapq.heappush(open_set,(math.hypot(si-gi,sj-gj)*self.grid,counter,start_id))
        closed=set()
        neighbors=[(di,dj) for di in (-1,0,1) for dj in (-1,0,1) if di or dj]
        while open_set:
            _,_,node=heapq.heappop(open_set)
            if node in closed:continue
            if node==goal_id:break
            closed.add(node)
            i,j=node
            x,y=self.to_world(i,j)
            h0=terrain(x,y)[0]
            for di,dj in neighbors:
                ni,nj=i+di,j+dj
                if not passable(ni,nj) or (ni,nj) in closed:continue
                if di and dj and (not passable(i+di,j) or not passable(i,j+dj)):continue
                nx,ny=self.to_world(ni,nj)
                dh=terrain(nx,ny)[0]-h0
                if not self.segment_safe((x,y),(nx,ny)):continue
                dist=math.hypot(nx-x,ny-y)
                step=math.hypot(dist,dh)+2*max(0.,dh)
                new_cost=costs[node]+step
                if new_cost<costs.get((ni,nj),float('inf')):
                    costs[(ni,nj)]=new_cost
                    parents[(ni,nj)]=node
                    counter+=1
                    heur=math.hypot(nx-self.goal[0],ny-self.goal[1])
                    heapq.heappush(open_set,(new_cost+heur,counter,(ni,nj)))
        if goal_id!=start_id and goal_id not in parents:
            raise RuntimeError('A* cannot find a safe route (do NOT drive straight through obstacles)')
        nodes=[goal_id]
        while nodes[-1]!=start_id:
            nodes.append(parents[nodes[-1]])
        nodes.reverse()
        raw=[tuple(start)]+[self.to_world(*n) for n in nodes[1:-1]]+[self.goal]
        smooth=[raw[0]]
        i=0
        while i<len(raw)-1:
            nxt=len(raw)-1
            while nxt>i+1 and not self.segment_safe(raw[i],raw[nxt]):nxt-=1
            smooth.append(raw[nxt]);i=nxt
        self.path=smooth
        return smooth

    def nearest_s_and_target(self, pos):
        path=self.path
        if len(path)<2:raise RuntimeError('A* path is empty')
        lengths=[math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:])]
        cumulative=[0.0]
        for length in lengths:cumulative.append(cumulative[-1]+length)
        near_d=float('inf'); near_s=0.0
        for j,((ax,ay),(bx,by)) in enumerate(zip(path,path[1:])):
            dx,dy=bx-ax,by-ay
            t=clamp(((pos[0]-ax)*dx+(pos[1]-ay)*dy)/max(dx*dx+dy*dy,1e-9),0,1)
            closest=(ax+t*dx,ay+t*dy)
            d=math.hypot(pos[0]-closest[0],pos[1]-closest[1])
            if d<near_d:near_d=d;near_s=cumulative[j]+t*lengths[j]
        return near_s,near_d,cumulative,lengths

    def target(self, pose):
        near_s,dev,cum,lengths=self.nearest_s_and_target((pose.x,pose.y))
        s=clamp(near_s+max(1.25,self.lookahead+0.5*abs(pose.v)),0,cum[-1])
        for i in range(len(lengths)):
            if s<=cum[i+1] or i==len(lengths)-1:
                t=clamp((s-cum[i])/max(lengths[i],1e-8),0,1)
                a,b=self.path[i:i+2]
                return (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])),dev
        return self.path[-1],dev

    def feedback(self,pose):
        tgt,dev=self.target(pose)
        dx=tgt[0]-pose.x;dy=tgt[1]-pose.y
        loc_y=-math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
        look=max(math.hypot(dx,dy),0.6)
        curvature=2*loc_y/(look*look)
        delta=clamp(math.atan(self.L*curvature),-self.max_delta,self.max_delta)
        dist=math.hypot(pose.x-self.goal[0],pose.y-self.goal[1])
        if dist<self.goal_tolerance:
            vref=0.0
            delta=0.0
        else:
            vref=min(self.max_speed, max(0.20, 0.7*dist))
            vref=min(vref, math.sqrt(0.7/max(abs(curvature),0.1)))
            vref=min(vref, max(0.18, 0.7*dist))
        _,hx,hy=terrain(pose.x,pose.y)
        grade=hx*math.cos(pose.yaw)+hy*math.sin(pose.yaw)
        # Signed drive force from speed tracking + rolling drag + gravity feedforward.
        a=clamp(1.4*(vref-pose.v),-1.6,1.6)
        force=clamp(self.eq_mass*a+self.c_roll*pose.v+self.mass*9.81*grade,
                    -self.max_F,self.max_F)
        # Optional hill-start assistance: the current robot is force-driven.
        # Extra torque must overcome static resistance when starting uphill.
        # This is a commanded motor force, NOT a guarantee of available traction.
        # AWD is preferable: rear-only could spin tires instead of climbing.
        if (self.hill_start_force > 0.0 and grade >= self.hill_start_min_grade
                and 0.0 <= pose.v < 0.15 and vref > 0.20
                and dist > self.goal_tolerance):
            force = max(force, self.hill_start_force)
        return force,delta,tgt,dist,dev,vref

    def step(self,pose,F,delta,dt):
        _,hx,hy=terrain(pose.x,pose.y)
        grade=hx*math.cos(pose.yaw)+hy*math.sin(pose.yaw)
        dv=(F-self.c_roll*pose.v-self.mass*9.81*grade)/self.eq_mass
        v=clamp(pose.v+dt*dv,-self.max_speed,self.max_speed)
        yaw=wrap_pi(pose.yaw+dt*v*math.tan(delta)/self.L)
        return Pose(pose.x+dt*v*math.cos(yaw),pose.y+dt*v*math.sin(yaw),yaw,v)

    def local_force_steer(self,pose):
        F0,steer0,target,dist,dev,vref=self.feedback(pose)
        if dist<self.goal_tolerance:
            # Active braking force until vehicle is nearly stopped. Zero isn't a brake on a slope.
            return (-clamp(self.eq_mass*1.4*pose.v,-self.max_F,self.max_F),0.0,dist,dev,vref)
        candidate_F=sorted(set(round(clamp(F0+shift,-self.max_F,self.max_F),3)
                               for shift in (-30.,0.,30.)))
        candidate_delta=sorted(set(round(clamp(steer0+shift,-self.max_delta,self.max_delta),4)
                                   for shift in (-0.12,0.,0.12)))
        best=None
        for F in candidate_F:
            for delta in candidate_delta:
                pred=pose; safe=True; pathcost=0.; mindist=float('inf')
                for _ in range(12):
                    pred=self.step(pred,F,delta,0.12)
                    if not self.safe_motion(pred.x,pred.y,pred.yaw):
                        safe=False;break
                    _,d_route,_,_=self.nearest_s_and_target((pred.x,pred.y))
                    pathcost+=d_route
                    for ox,oy,rad in self.obstacles:
                        mindist=min(mindist,math.hypot(pred.x-ox,pred.y-oy)-rad-self.infl)
                if not safe:continue
                d_goal=math.hypot(pred.x-self.goal[0],pred.y-self.goal[1])
                d_tgt=math.hypot(pred.x-target[0],pred.y-target[1])
                score=1.2*d_tgt+0.15*d_goal+0.22*pathcost/12-0.8*pred.v+0.001*abs(F-F0)+0.4*abs(delta-steer0)
                if mindist<float('inf'):score+=0.15/max(mindist+0.2,0.12)
                if best is None or score<best[0]:best=(score,F,delta)
        if best is None:
            return (0.0,0.0,dist,dev,0.0)  # fail-safe without known safe rollout
        return best[1],best[2],dist,dev,vref