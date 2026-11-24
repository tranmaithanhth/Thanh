#!/usr/bin/env python3
"""Ground-displacement-driven traction diagnosis/assist for Gazebo Ackermann.

Velocity from odom can oscillate while the rear axle is effectively stuck.
Do not use abs(odom.twist.linear.x) alone to declare useful travel.  This
module observes XY movement over a window, then requests a bounded motor
force FLOOR when progress stalls, without modifying collision constraints.

Simulation only; wheel/road force and slip remain model approximations.
"""
from collections import deque
import math


class DisplacementWatchdog:
    def __init__(self, window_s=1.6, threshold_m=0.045,
                 speed_threshold_m_s=0.15):
        self.window_s=float(window_s)
        self.threshold_m=float(threshold_m)
        self.speed_threshold_m_s=float(speed_threshold_m_s)
        if self.window_s<=0 or self.threshold_m<=0 or self.speed_threshold_m_s<0:
            raise ValueError('Invalid progress watchdog thresholds')
        self.samples=deque()
        self.stalled_since=None
        self.xy_speed=math.nan
        self.xy_displacement=0.0
        self.last_reference_v=0.

    def reset(self):
        self.samples.clear()
        self.stalled_since=None
        self.xy_speed=math.nan
        self.xy_displacement=0.

    def update(self,now,x,y,vref,enabled=True):
        now=float(now);x=float(x);y=float(y);vref=float(vref)
        if not all(math.isfinite(v) for v in (now,x,y,vref)):
            self.reset()
            return 0.0
        self.last_reference_v=vref
        if not enabled or abs(vref)<self.speed_threshold_m_s:
            self.reset()
            return 0.0
        if self.samples and now<=self.samples[-1][0]:
            self.reset()
        self.samples.append((now,x,y))
        # Keep one record just older than the lookback window, so that the
        # reported distance is over a complete window even at 10 Hz.
        while (len(self.samples)>2 and
               self.samples[1][0] < now-self.window_s):
            self.samples.popleft()
        old_t,old_x,old_y=self.samples[0]
        elapsed=now-old_t
        if elapsed<self.window_s:
            self.stalled_since=None
            return 0.0
        displacement=math.hypot(x-old_x,y-old_y)
        self.xy_displacement=displacement
        self.xy_speed=displacement/max(elapsed,1e-6)
        if displacement>self.threshold_m:
            self.stalled_since=None
            return 0.0
        if self.stalled_since is None:
            self.stalled_since=now
        return max(0.0,now-self.stalled_since)

    def force_floor(self,now,x,y,vref,enabled=True,start_N=65.0,
                    ramp_N_per_s=22.0,max_N=145.0,
                    min_stall_s=0.35):
        stalled_s=self.update(now,x,y,vref,enabled=enabled)
        if self.stalled_since is None or stalled_s<min_stall_s:
            return 0.0
        if not (0<=start_N<=max_N and ramp_N_per_s>=0):
            raise ValueError('Invalid bounded traction recovery profile')
        return min(max_N,start_N+ramp_N_per_s*(
            stalled_s-min_stall_s))
