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
        # Extra buffer for GLOBAL routes only. Local safety still checks
        # the real inflated footprint, not an artificially larger body.
        self.route_margin = float(config.get('route_clearance_margin_m', 0.0))
        if self.route_margin < 0.0:
            raise ValueError('route_clearance_margin_m must be nonnegative')
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
        # Do not clamp rollout at the DESIRED speed. The vehicle can overshoot
        # its reference, so the prediction model must preserve that overspeed.
        self.physical_max_speed = float(config.get('physical_max_speed_m_s', max(1.45, self.max_speed)))
        self.min_progress_force = float(config.get('min_progress_force_N', 0.0))
        self.speed_tracking_cost_weight = float(config.get('speed_tracking_cost_weight', 4.0))
        self.overspeed_cost_weight = float(config.get('overspeed_cost_weight', 14.0))
        if self.physical_max_speed < self.max_speed or not (0 <= self.min_progress_force <= self.max_F):
            raise ValueError('Invalid physical_max_speed_m_s or min_progress_force_N')
        self.goal_tolerance = float(config['goal_tolerance_m'])
        self.lookahead = float(config['lookahead_m'])
        if self.grid <= 0 or self.max_F <= 0 or self.L <= 0 or self.goal_tolerance <= 0:
            raise ValueError('Invalid mission model parameters')
        self.nx = round((self.bounds[1]-self.bounds[0])/self.grid)+1
        self.ny = round((self.bounds[3]-self.bounds[2])/self.grid)+1
        self.occupied = [[not self.safe_xy(self.to_world(i,j)[0],self.to_world(i,j)[1],
                                            margin=self.route_margin)
                          for i in range(self.nx)] for j in range(self.ny)]
        self.path = []
        # Direction of travel is selected from ACTUAL pose/yaw, not just XY.
        # It stays latched while rolling to prevent rapid gear reversal.
        self.drive_direction = 1
        self.direction_initialized = False

    def to_grid(self, x, y):
        return (round((x-self.bounds[0])/self.grid), round((y-self.bounds[2])/self.grid))

    def to_world(self, i, j):
        return (self.bounds[0]+i*self.grid, self.bounds[2]+j*self.grid)

    def safe_xy(self,x,y,margin=0.0):
        if x <= self.bounds[0]+0.35 or x >= self.bounds[1]-0.35 or y <= self.bounds[2]+0.35 or y >= self.bounds[3]-0.35:
            return False
        _, hx, hy = terrain(x,y)
        if math.hypot(hx,hy)>self.max_grade:
            return False
        return all(math.hypot(x-ox,y-oy) > rad+self.infl+margin for ox,oy,rad in self.obstacles)

    def safe_motion(self,x,y,yaw):
        if not self.safe_xy(x,y): return False
        _,hx,hy=terrain(x,y)
        grade=hx*math.cos(yaw)+hy*math.sin(yaw)
        cross=-hx*math.sin(yaw)+hy*math.cos(yaw)
        return abs(grade)<=self.max_grade and abs(cross)<=self.cross_grade

    def segment_safe(self,a,b,margin=None):
        # Global route simplification must retain route clearance. A starting
        # connector may use physical (inflated) clearance without the extra
        # route margin; it must still never intersect a known obstacle.
        if margin is None:margin=self.route_margin
        distance = math.hypot(b[0]-a[0], b[1]-a[1])
        steps = max(1, math.ceil(distance/0.10))
        for k in range(steps+1):
            t=k/steps
            if not self.safe_xy(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),
                                margin=margin):return False
        return True

    def astar(self,start):
        if not self.safe_xy(*start):
            raise ValueError(f'Start {start} is inside a known inflated obstacle, outside bounds, or too steep. '
                             'Move the vehicle to a physically safe pose.')
        # The start can be outside the EXTRA 0.35m global-route margin while
        # still physically safe. Connect to a route-clear cell using the true
        # inflated physical footprint, without dropping collision checks.
        start_has_route_margin=self.safe_xy(*start,margin=self.route_margin)
        if not self.safe_xy(*self.goal, margin=self.route_margin):
            raise ValueError(f'Goal {self.goal} is outside the safe planning corridor.')
        self.drive_direction=1
        self.direction_initialized=False
        if self.segment_safe(start,self.goal):
            self.path=[tuple(start),self.goal]
            return self.path
        si,sj=self.to_grid(*start)
        gi,gj=self.to_grid(*self.goal)
        def passable(i,j):
            return 0<=i<self.nx and 0<=j<self.ny and not self.occupied[j][i]
        if not passable(gi,gj):
            raise ValueError('Goal snapped into route obstacle; reduce grid resolution')
        needs_connector=(not start_has_route_margin) or (not passable(si,sj))
        goal_id=(gi,gj)
        costs={};parents={};open_set=[];counter=0
        if needs_connector:
            # Virtual grid start; seed multiple safe, short physical connectors
            # rather than incorrectly treating the nearest blocked grid cell
            # as drivable. Entire rest of route obeys the extra clearance.
            start_id=(-1,-1)
            options=[]
            for j in range(max(0,sj-9),min(self.ny,sj+10)):
                for i in range(max(0,si-9),min(self.nx,si+10)):
                    if not passable(i,j):continue
                    xy=self.to_world(i,j)
                    d=math.hypot(xy[0]-start[0],xy[1]-start[1])
                    if d>2.8 or not self.segment_safe(start,xy,margin=0.0):
                        continue
                    options.append((d,i,j))
            options.sort()
            if not options:
                raise ValueError('No collision-free connection from current pose '
                                 'to any route-clear grid cell; move to open ground')
            for d,i,j in options[:16]:
                node=(i,j)
                costs[node]=d
                parents[node]=start_id
                counter+=1
                x,y=self.to_world(i,j)
                h=math.hypot(x-self.goal[0],y-self.goal[1])
                heapq.heappush(open_set,(d+h,counter,node))
        else:
            start_id=(si,sj)
            costs[start_id]=0.0
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
        if needs_connector:
            # First leg is physically safe, not route-margin safe; never
            # simplify it into a long shortcut across the obstacle buffer.
            if len(raw)<3 or not self.segment_safe(raw[0],raw[1],margin=0.0):
                raise RuntimeError('A* initial physical connector invalid')
            smooth=[raw[0],raw[1]]
            i=1
        else:
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

    def axle_grades(self,pose):
        """Longitudinal slopes at rear P and front axle, L metres ahead."""
        c=math.cos(pose.yaw); st=math.sin(pose.yaw)
        _,rhx,rhy=terrain(pose.x,pose.y)
        _,fhx,fhy=terrain(pose.x+self.L*c,pose.y+self.L*st)
        return rhx*c+rhy*st, fhx*c+fhy*st

    def grade_for_dynamics(self,pose):
        # Half-load per axle proxy; MATLAB rear-axle point alone misses
        # front contact with ramp during approach.
        rear,front=self.axle_grades(pose)
        return 0.5*(rear+front)

    def hill_start_force_floor(self,pose,vref,dist):
        rear,front=self.axle_grades(pose)
        full_below=float(self.c.get('hill_start_full_force_below_m_s',0.20))
        release=float(self.c.get('hill_start_release_speed_m_s',0.45))
        if release<=full_below:
            raise ValueError('hill-start release speed must exceed full-force threshold')
        direction=1 if vref>0 else -1
        travel_grade=max(direction*rear,direction*front)
        along_speed=direction*pose.v
        if not (self.hill_start_force>0.0 and
                travel_grade>=self.hill_start_min_grade and
                0.0<=along_speed<release and abs(vref)>0.20 and
                dist>self.goal_tolerance):
            return 0.0
        # Smoothly release torque support in the current travel direction.
        factor=clamp((release-along_speed)/(release-full_below),0.0,1.0)
        return self.hill_start_force*factor

    def hill_start_active(self,pose,vref,dist):
        return self.hill_start_force_floor(pose,vref,dist)>0.0

    def rollout_safe(self,pose,F,delta,horizon_s=0.45):
        """Short-horizon collision/grade/route-deviation guard for boost."""
        n=max(1,math.ceil(horizon_s/0.05))
        pred=pose
        for _ in range(n):
            pred=self.step(pred,F,delta,horizon_s/n)
            if not self.safe_motion(pred.x,pred.y,pred.yaw):
                return False
            _,dev,_,_=self.nearest_s_and_target((pred.x,pred.y))
            if dev>float(self.c.get('replan_after_deviation_m',2.0)):
                return False
        return True

    def choose_drive_direction(self,pose,target):
        """Prefer reverse if the route lies behind the current heading.

        This is a simple heading-aware forward/reverse adapter to geometric
        A*, not a full Hybrid A* or multi-point manoeuvre planner.
        Changes direction only when nearly stopped to avoid violent reversal.
        """
        bearing=math.atan2(target[1]-pose.y,target[0]-pose.x)
        err=abs(wrap_pi(bearing-pose.yaw))
        if not self.direction_initialized:
            self.drive_direction=-1 if err>math.radians(105) else 1
            self.direction_initialized=True
        elif abs(pose.v)<0.15:
            if self.drive_direction>0 and err>math.radians(125):
                self.drive_direction=-1
            elif self.drive_direction<0 and err<math.radians(55):
                self.drive_direction=1
        return self.drive_direction

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
            direction=self.choose_drive_direction(pose,tgt)
            speed=min(self.max_speed, max(0.20, 0.7*dist))
            speed=min(speed, math.sqrt(0.7/max(abs(curvature),0.1)))
            speed=min(speed, max(0.18, 0.7*dist))
            # Starting with the route sideways requires slow alignment first.
            # Reverse speed is conservative and physically bounded by sign.
            if direction<0:
                speed=min(speed,float(self.c.get('reverse_cruise_speed_m_s',0.35)))
            bearing_err=abs(wrap_pi(math.atan2(tgt[1]-pose.y,tgt[0]-pose.x)
                                    -pose.yaw))
            if abs(math.cos(bearing_err))<0.40:
                speed=min(speed,float(self.c.get('heading_alignment_speed_m_s',0.22)))
            vref=direction*speed
        grade=self.grade_for_dynamics(pose)
        # Signed drive force from speed tracking + rolling drag + gravity feedforward.
        a=clamp(1.4*(vref-pose.v),-1.6,1.6)
        force=clamp(self.eq_mass*a+self.c_roll*pose.v+self.mass*9.81*grade,
                    -self.max_F,self.max_F)
        # Optional hill-start assistance: the current robot is force-driven.
        # Extra torque must overcome static resistance when starting uphill.
        # This is a commanded motor force, NOT a guarantee of available traction.
        # AWD is preferable: rear-only could spin tires instead of climbing.
        hill_floor=self.hill_start_force_floor(pose,vref,dist)
        if hill_floor>0.0:
            # Force sign must follow the chosen travel direction.
            direction=1 if vref>0 else -1
            force=direction*max(direction*force,hill_floor)
        return force,delta,tgt,dist,dev,vref

    def step(self,pose,F,delta,dt):
        grade=self.grade_for_dynamics(pose)
        dv=(F-self.c_roll*pose.v-self.mass*9.81*grade)/self.eq_mass
        # Physical dynamics can exceed cruise reference; preserve overspeed.
        v=clamp(pose.v+dt*dv,-self.physical_max_speed,self.physical_max_speed)
        yaw=wrap_pi(pose.yaw+dt*v*math.tan(delta)/self.L)
        return Pose(pose.x+dt*v*math.cos(yaw),pose.y+dt*v*math.sin(yaw),yaw,v)

    def local_force_steer(self,pose):
        F0,steer0,target,dist,dev,vref=self.feedback(pose)
        if dist<self.goal_tolerance:
            # Active braking force until vehicle is nearly stopped. Zero isn't a brake on a slope.
            return (-clamp(self.eq_mass*1.4*pose.v,-self.max_F,self.max_F),0.0,dist,dev,vref)
        # The original sampler considered F0-30 even at low speed and could
        # rank a standstill/negative-force rollout above useful forward motion.
        # Search steering/drive options more broadly, but NEVER ignore collision
        # and grade checks: if none are safe we still stop.
        force_shifts=(-45.,-20.,0.,20.,45.)
        candidate_F=sorted(set(round(clamp(F0+shift,-self.max_F,self.max_F),3)
                               for shift in force_shifts))
        # The old MPC could reject the hill-start torque due to its idealized
        # speed model and send 24N while the front axle was climbing.
        hill_floor=self.hill_start_force_floor(pose,vref,dist)
        if hill_floor>0.0:
            direction=1 if vref>0 else -1
            candidate_F=[F for F in candidate_F if direction*F>=hill_floor-1e-6]
            if not candidate_F:
                candidate_F=[direction*hill_floor]
        candidate_delta=sorted(set(round(clamp(steer0+shift,-self.max_delta,self.max_delta),4)
                                   for shift in (-0.18,-0.09,0.,0.09,0.18)))
        best=None
        start_progress=self.nearest_s_and_target((pose.x,pose.y))[0]
        for F in candidate_F:
            # When nearly stationary and target speed is positive, commanding
            # reverse torque to "avoid" a route tangent produces deadlock.
            # A genuine obstacle can still force zero if every forward rollout
            # fails the safety checks.
            if ((1 if vref>=0 else -1)*pose.v < 0.25 and abs(vref)>0.25 and
                    (1 if vref>=0 else -1)*F < self.min_progress_force):
                continue
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
                end_progress=self.nearest_s_and_target((pred.x,pred.y))[0]
                progress=max(-1.,min(2.,end_progress-start_progress))
                # Replaces the old '-0.8*pred.v' which wrongly rewarded
                # overspeed and treated crossing 0.75m/s as cost-free.
                speed_error=abs(pred.v-vref)
                # Overspeed is signed along intended motion; in reverse,
                # -0.50m/s is faster than the -0.35m/s reference.
                overspeed=max(0.,(1 if vref>=0 else -1)*(pred.v-vref))
                score=(1.2*d_tgt+0.15*d_goal+0.22*pathcost/12
                       +self.speed_tracking_cost_weight*speed_error
                       +self.overspeed_cost_weight*overspeed*overspeed
                       -0.45*progress
                       +0.002*abs(F-F0)+0.4*abs(delta-steer0))
                if mindist<float('inf'):
                    score+=0.15/max(mindist+0.2,0.12)
                if best is None or score<best[0]:best=(score,F,delta)
        if best is None:
            return (0.0,0.0,dist,dev,0.0)  # fail-safe without known safe rollout
        return best[1],best[2],dist,dev,vref