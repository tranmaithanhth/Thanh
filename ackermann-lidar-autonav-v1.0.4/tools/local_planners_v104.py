#!/usr/bin/env python3
"""Ackermann DWA and reduced TEB local planners for ROS 2 v1.0.4.

Follows the user's MATLAB Terrain3D v1.0.4 architecture:
 - DWA: 6 reachable speeds x 9 reachable steering angles, ~2.5s rollout.
 - Reduced TEB: seven XY nodes and positive inter-node times, with a
   dependency-free bounded Nelder-Mead optimizer and DWA safety fallback.
The original ROS model drives via [motor_force_N, steering_rad], NOT cmd_vel.
Both methods use Mission.step() with slope/gravity and safe_motion()
including *discovered* nearby obstacles. Not the full ROS teb_local_planner.
"""
from __future__ import annotations

import math
import time

from lidar_planner import DynamicMission
from planner_core import Pose, clamp, wrap_pi, terrain

MODES=("DWA","TEB","ROLLOUT")


def linspace(a,b,n):
    if n<2:return [float(a)]
    return [float(a+(b-a)*i/(n-1)) for i in range(n)]


def nearest_s_point(mission,s):
    """Arc-length point on the current A* polyline."""
    path=mission.path
    if len(path)<2:raise ValueError("A* path empty")
    s=max(0.,float(s))
    for a,b in zip(path,path[1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if s<=length or b==path[-1]:
            t=clamp(s/max(length,1e-9),0.,1.)
            return (a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t)
        s-=length
    return path[-1]


def nelder_mead(fun, x0, scales, max_evals=85):
    """Pure Python bounded-budget fminsearch analogue (not SciPy required)."""
    n=len(x0)
    vertices=[list(x0)]
    for j in range(n):
        v=list(x0);v[j]+=float(scales[j]);vertices.append(v)
    values=[fun(v) for v in vertices]
    neval=len(vertices)
    while neval<max_evals:
        order=sorted(range(n+1),key=lambda i:values[i])
        vertices=[vertices[i] for i in order]
        values=[values[i] for i in order]
        center=[sum(vertices[i][j] for i in range(n))/n for j in range(n)]
        worst=vertices[-1]
        def at(alpha):
            return [center[j]+alpha*(center[j]-worst[j]) for j in range(n)]
        xr=at(1.)
        fr=fun(xr);neval+=1
        if fr<values[0] and neval<max_evals:
            xe=at(2.);fe=fun(xe);neval+=1
            vertices[-1],values[-1]=(xe,fe) if fe<fr else (xr,fr)
        elif fr<values[-2]:
            vertices[-1],values[-1]=xr,fr
        else:
            outside=fr<values[-1]
            xc=at(.5) if outside else at(-.5)
            fc=fun(xc);neval+=1
            if fc<(fr if outside else values[-1]):
                vertices[-1],values[-1]=xc,fc
            elif neval+n<=max_evals:
                best=vertices[0]
                for i in range(1,n+1):
                    vertices[i]=[(best[j]+vertices[i][j])*.5 for j in range(n)]
                    values[i]=fun(vertices[i]);neval+=1
            else:
                break
        if values[-1]-values[0]<1e-5:break
    idx=min(range(len(values)),key=lambda i:values[i])
    return list(vertices[idx]),float(values[idx]),neval


class LocalPlanningMission(DynamicMission):
    """A* plus the selected local trajectory generator.

    Public local_force_steer(Pose) signature remains identical to the
    previously working ForceNavigator, to preserve goal/stall/recovery logic.
    """
    def __init__(self,cfg):
        super().__init__(cfg)
        self.mode=str(cfg.get('local_planner','DWA')).upper()
        if self.mode not in MODES:raise ValueError('Unknown local planner '+self.mode)
        self.local_path=[]
        self.local_cost=math.nan
        self.fallback_active=False
        self.last_delta=0.
        self.last_reference_v=0.
        self._teb_last_time=-1e9
        self._teb_last_result=None

    def _force(self,pose,vref,dist,amax=1.3):
        """Convert v_ref to motor force using the SAME slope dynamics as ROS."""
        grade=self.grade_for_dynamics(pose)
        a=clamp(1.4*(vref-pose.v),-amax,amax)
        F=clamp(self.eq_mass*a+self.c_roll*pose.v+
                self.mass*9.81*grade,-self.max_F,self.max_F)
        if abs(vref)>0.04 and (vref*pose.v)<=abs(vref)*0.25:
            # Avoid stalling at low speed due to sampling sub-15N force.
            F=math.copysign(max(abs(F),self.min_progress_force),vref)
        floor=self.hill_start_force_floor(pose,vref,dist)
        if floor>0:
            sign=1 if vref>0 else -1
            F=sign*max(sign*F,floor)
        return clamp(F,-self.max_F,self.max_F)

    def _clearance(self,x,y):
        all_obs=self.obstacles+self.dynamic_obstacles
        if not all_obs:return 8.0
        return min(math.hypot(x-ox,y-oy)-rad-self.infl
                   for ox,oy,rad in all_obs)

    def _simulate(self,pose,vref,steer,dist,horizon,dt):
        """Continuous-force simulated rollout under bicycle/slope dynamics."""
        n=max(1,math.ceil(horizon/dt))
        state=pose
        states=[pose]
        err=0.;cl_min=8.;grade_pen=0.;torque_pen=0.
        for _ in range(n):
            F=self._force(state,vref,dist)
            state=self.step(state,F,steer,horizon/n)
            if not self.safe_motion(state.x,state.y,state.yaw):
                return None
            _,dev,_,_=self.nearest_s_and_target((state.x,state.y))
            err+=dev
            cl_min=min(cl_min,self._clearance(state.x,state.y))
            grade_pen+=max(0.,self.grade_for_dynamics(state))**2
            torque_pen+=abs(F)/max(self.max_F,1.)
            states.append(state)
        return states,err/n,cl_min,grade_pen/n,torque_pen/n

    def _dwa(self,pose,feedback):
        Fbase,deltabase,target,dist,dev,vguide=feedback
        direction=self.choose_drive_direction(pose,target)
        dt_control=1./float(self.c['planner_control_hz'])
        amax=float(self.c.get('dwa_max_accel_m_s2',1.35))
        steer_rate=float(self.c.get('max_steer_rate_rad_s',0.95))
        signed_v=direction*pose.v
        speed_cap=self.max_speed if direction>0 else min(
            self.max_speed,float(self.c.get('reverse_cruise_speed_m_s',0.35)))
        vlo=max(0.,signed_v-amax*dt_control)
        vhi=min(speed_cap,max(0.,signed_v+amax*dt_control))
        if vlo>vhi:vlo=vhi
        # MATLAB: six velocities plus zero, nine steering samples.
        vset=sorted(set([0.]+[round(v,5) for v in linspace(vlo,vhi,6)]))
        dlo=max(-self.max_delta,self.last_delta-steer_rate*dt_control)
        dhi=min(self.max_delta,self.last_delta+steer_rate*dt_control)
        if dlo>dhi:dlo=dhi
        dset=linspace(dlo,dhi,9)
        start_s=self.nearest_s_and_target((pose.x,pose.y))[0]
        best=None
        for speed in vset:
            vref=direction*speed
            for steer in dset:
                res=self._simulate(
                    pose,vref,steer,dist,
                    float(self.c.get('dwa_horizon_s',2.5)),
                    float(self.c.get('dwa_rollout_dt_s',0.2)))
                if res is None:continue
                states,route_err,clearance,grade_cost,torque_cost=res
                end=states[-1]
                progress=direction*(
                    self.nearest_s_and_target((end.x,end.y))[0]-start_s)
                heading=math.atan2(target[1]-end.y,target[0]-end.x)
                herr=abs(wrap_pi(heading-end.yaw))
                cost=(1.6*math.hypot(end.x-target[0],end.y-target[1])
                      +.04*math.hypot(end.x-self.goal[0],end.y-self.goal[1])
                      +.19*route_err +.45*herr
                      +1.15/max(clearance+.35,.12)
                      -.95*max(0.,direction*end.v)
                      +.05*abs(steer)+.12*abs(steer-deltabase)
                      +.06*abs(vref-vguide)+.1*grade_cost+.08*torque_cost
                      -1.1*progress)
                if speed<0.02 and dist>self.goal_tolerance:
                    cost+=.6
                if best is None or cost<best[0]:
                    best=(cost,vref,steer,states)
        if best is None:return None
        cost,vref,steer,states=best
        # Sample the predicted trajectory to N=7 points for the HMI.
        pts=[(states[round(i*(len(states)-1)/6)].x,
              states[round(i*(len(states)-1)/6)].y) for i in range(7)]
        F=self._force(pose,vref,dist)
        return (F,steer,dist,dev,vref,pts,cost)

    def _reference_band(self,pose,n,lookahead):
        s,_,_,_=self.nearest_s_and_target((pose.x,pose.y))
        pts=[(pose.x,pose.y)]
        for i in range(1,n):
            pts.append(nearest_s_point(self,s+lookahead*i/(n-1)))
        return pts

    def _unpack_band(self,z,pose,endpoint,n):
        mid=[(z[2*i],z[2*i+1]) for i in range(n-2)]
        times=[math.exp(clamp(v,-2.,2.)) for v in z[2*(n-2):]]
        return [(pose.x,pose.y)]+mid+[endpoint],times

    def _teb_cost(self,z,pose,endpoint,n,guide):
        pts,times=self._unpack_band(z,pose,endpoint,n)
        segments=[(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts,pts[1:])]
        ds=[max(1e-6,math.hypot(dx,dy)) for dx,dy in segments]
        headings=[math.atan2(dy,dx) for dx,dy in segments]
        velocities=[d/t for d,t in zip(ds,times)]
        turns=[wrap_pi(headings[i+1]-headings[i])
               for i in range(n-2)]
        curvature=[
            turns[i]/max(.05,(ds[i]+ds[i+1])*.5)
            for i in range(n-2)]
        acc=[(velocities[i+1]-velocities[i])/
             max(.1,(times[i]+times[i+1])*.5)
             for i in range(n-2)]
        max_curvature=math.tan(self.max_delta)/self.L
        route_cost=0.;obstacle_cost=0.;slope_cost=0.
        for i in range(n-1):
            for fraction in (0.,.5,1.):
                x=pts[i][0]+fraction*segments[i][0]
                y=pts[i][1]+fraction*segments[i][1]
                th=headings[i]
                _,dev,_,_=self.nearest_s_and_target((x,y))
                route_cost+=dev*dev/3.
                clearance=self._clearance(x,y)
                obstacle_cost+=max(0.,.55-clearance)**2/3.
                if not self.safe_xy(x,y):
                    obstacle_cost+=12.
                _,hx,hy=terrain(x,y)
                grade=hx*math.cos(th)+hy*math.sin(th)
                cross=-hx*math.sin(th)+hy*math.cos(th)
                slope_cost+=(5.*max(0.,grade)**2+
                    70.*max(0.,abs(grade)-self.max_grade)**2+
                    70.*max(0.,abs(cross)-self.cross_grade)**2)/3.
        initial=wrap_pi(headings[0]-pose.yaw)
        cost=(.40*sum(times)+.24*route_cost
              +.65*sum(t*t for t in turns)
              +120.*obstacle_cost+slope_cost
              +95.*sum(max(0.,abs(k)-max_curvature)**2 for k in curvature)
              +35.*sum(max(0.,v-self.max_speed)**2 for v in velocities)
              +4.*sum(max(0.,abs(a)-float(
                    self.c.get('dwa_max_accel_m_s2',1.35)))**2 for a in acc)
              +1.8*initial*initial
              +.08*sum((times[i+1]-times[i])**2 for i in range(n-2))
              +.15*wrap_pi(headings[0]-
                  (pose.yaw+.5*math.tan(guide[1])/self.L))**2)
        return cost if math.isfinite(cost) else 1e12

    def _teb_is_safe(self,pose,pts):
        # Validate the actual curve, not just the optimizer's soft penalties.
        if len(pts)<2:return False
        prev_theta=pose.yaw
        for a,b in zip(pts,pts[1:]):
            dx,dy=b[0]-a[0],b[1]-a[1]
            length=math.hypot(dx,dy)
            if length<.04:return False
            th=math.atan2(dy,dx)
            if abs(wrap_pi(th-prev_theta))>1.1 and length<.8:
                return False
            prev_theta=th
            for k in range(max(2,math.ceil(length/.13))+1):
                t=k/max(2,math.ceil(length/.13))
                if not self.safe_motion(a[0]+t*dx,a[1]+t*dy,th):
                    return False
        return True

    def _teb(self,pose,feedback):
        _,_,_,dist,dev,vguide=feedback
        if vguide<=0:return None  # Reverse and hard turning use safe DWA.
        n=int(self.c.get('teb_path_nodes',7))
        if n!=7:raise ValueError("This optimized TEB currently uses seven nodes")
        pts=self._reference_band(
            pose,n,float(self.c.get('teb_lookahead_m',3.6)))
        endpoint=pts[-1]
        mid=[value for p in pts[1:-1] for value in p]
        times=[max(.23,math.hypot(b[0]-a[0],b[1]-a[1])/
                    max(.30,abs(vguide))) for a,b in zip(pts,pts[1:])]
        z0=mid+[math.log(t) for t in times]
        scales=[.18]*(2*(n-2))+[.12]*(n-1)
        best,cost,_=nelder_mead(
            lambda z:self._teb_cost(z,pose,endpoint,n,(vguide,feedback[1])),
            z0,scales,int(self.c.get('teb_max_evals',85)))
        optimized,dt=self._unpack_band(best,pose,endpoint,n)
        if not self._teb_is_safe(pose,optimized):return None
        # MATLAB reduced TEB carrot at third pose, pure-pursuit conversion.
        x,y=optimized[min(2,n-1)]
        dx,dy=x-pose.x,y-pose.y
        look=max(math.hypot(dx,dy),.35)
        local_y=-math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
        curvature=clamp(2*local_y/(look*look),
                        -math.tan(self.max_delta)/self.L,
                        math.tan(self.max_delta)/self.L)
        steer=clamp(math.atan(self.L*curvature),
                    -self.max_delta,self.max_delta)
        # Honor motor steering-rate envelope (actual wheel feedback
        # unavailable here, previous command is the conservative proxy).
        max_d=float(self.c.get('max_steer_rate_rad_s',0.95)) / float(
            self.c['planner_control_hz'])
        steer=clamp(steer,self.last_delta-max_d,self.last_delta+max_d)
        speeds=[math.hypot(b[0]-a[0],b[1]-a[1])/max(.1,t)
                for a,b,t in zip(optimized,optimized[1:],dt)]
        speed=clamp(sum(speeds[:2])/2.,.05,self.max_speed)
        speed=min(speed,math.sqrt(1.0/max(abs(curvature),.1)),
                  .9*dist)
        F=self._force(pose,speed,dist)
        # Final ROS physics rollout: never trust soft TEB penalties alone.
        sim=self._simulate(
            pose,speed,steer,dist,
            float(self.c.get('teb_verify_horizon_s',1.2)),.12)
        if sim is None:return None
        return F,steer,dist,dev,speed,optimized,cost

    def local_force_steer(self,pose):
        if not self.path:raise RuntimeError("No global A* route")
        distance=math.hypot(pose.x-self.goal[0],pose.y-self.goal[1])
        if distance<self.goal_tolerance:
            self.local_path=[(pose.x,pose.y)]
            self.fallback_active=False
            return super().local_force_steer(pose)
        if self.mode=="ROLLOUT":
            self.local_path=[]
            self.fallback_active=False
            return super().local_force_steer(pose)
        guide=self.feedback(pose)
        self.fallback_active=False
        chosen=None
        now=time.monotonic()
        if self.mode=="TEB":
            if now-self._teb_last_time>=float(self.c.get('teb_period_s',.6)):
                self._teb_last_result=self._teb(pose,guide)
                self._teb_last_time=now
            if self._teb_last_result is not None:
                cached=self._teb_last_result
                # Recheck nearby obstacles and slope each tick before reuse.
                if self._teb_is_safe(pose,cached[5]) and self.rollout_safe(
                        pose,cached[0],cached[1],horizon_s=.45):
                    chosen=cached
            if chosen is None:
                self.fallback_active=True
        if chosen is None:
            chosen=self._dwa(pose,guide)
        if chosen is None:
            # Preserve the previously demonstrated working force sampler
            # as a collision/grade-checked failsafe, never drive blindly.
            self.fallback_active=True
            cmd=super().local_force_steer(pose)
            self.local_path=[]
            self.local_cost=math.inf
            self.last_delta=cmd[1]
            self.last_reference_v=cmd[4]
            return cmd
        force,delta,dist,dev,vref,pts,cost=chosen
        self.local_path=list(pts)
        self.local_cost=cost
        self.last_delta=delta
        self.last_reference_v=vref
        return force,delta,dist,dev,vref
