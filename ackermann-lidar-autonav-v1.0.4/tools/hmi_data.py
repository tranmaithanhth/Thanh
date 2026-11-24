#!/usr/bin/env python3
"""ROS-free HMI telemetry schema, CSV I/O and MATLAB comparison calculations.

The Gazebo force plugin reports MOTOR torque commands, not road contact forces.
Match MATLAB's rear-axle pose xP,yP with Gazebo's shifted vehicle-base origin.
"""
from __future__ import annotations
import bisect
import csv
import math
from pathlib import Path

FIELDS=(
    "t_s", "sim_time_s", "x_rear_m", "y_rear_m", "x_body_m", "y_body_m",
    "z_body_m", "yaw_deg", "roll_deg", "pitch_deg",
    "v_m_s", "yaw_rate_rad_s", "F_cmd_N",
    "delta_ref_deg", "delta_actual_deg",
    "omega_rl_rad_s", "omega_rr_rad_s", "omega_fl_rad_s", "omega_fr_rad_s",
    "tau_rl_Nm", "tau_rr_Nm", "tau_fl_Nm", "tau_fr_Nm", "tau_total_Nm",
    "wheel_surface_avg_m_s", "road_grade_deg", "route_error_m",
    "lidar_front_m", "lidar_rear_m", "lidar_age_s", "lidar_beams",
    "goal_error_m", "mode"
)
COMPARED=(
    ("x_rear_m","X cầu sau (m)"),
    ("y_rear_m","Y cầu sau (m)"),
    ("v_m_s","Vận tốc dọc (m/s)"),
    ("yaw_deg","Hướng yaw (°)"),
    ("delta_actual_deg","Góc lái tương đương (°)"),
    ("tau_total_Nm","Tổng mômen động cơ (Nm)"),
    ("F_cmd_N","Lực dẫn động quy đổi (N)"),
)

def numeric(v):
    try:
        v=float(v)
        return v if math.isfinite(v) else math.nan
    except (ValueError,TypeError):
        return math.nan

def quaternion_rpy(qx,qy,qz,qw):
    """Gazebo world orientation to roll pitch yaw (radians)."""
    sinr=2.0*(qw*qx+qy*qz)
    cosr=1.0-2.0*(qx*qx+qy*qy)
    roll=math.atan2(sinr,cosr)
    sinp=max(-1.0,min(1.0,2.0*(qw*qy-qz*qx)))
    pitch=math.asin(sinp)
    yaw=math.atan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy*qy+qz*qz))
    return roll,pitch,yaw

def distance_to_path(x,y,points):
    """Shortest planar distance to a published A* polyline; NaN if absent."""
    if not points:return math.nan
    if len(points)==1:return math.hypot(x-points[0][0],y-points[0][1])
    best=math.inf
    for (ax,ay),(bx,by) in zip(points,points[1:]):
        dx,dy=bx-ax,by-ay
        length2=dx*dx+dy*dy
        t=max(0.0,min(1.0,((x-ax)*dx+(y-ay)*dy)/length2)) if length2>1e-12 else 0.0
        best=min(best,math.hypot(x-(ax+t*dx),y-(ay+t*dy)))
    return best

def force_state_values(data):
    """Plugin layout [F, ref δ, actual δ, RL, RR, v, FL, FR, τR, τF]."""
    out={k:math.nan for k in (
        "F_cmd_N","delta_ref_deg","delta_actual_deg",
        "omega_rl_rad_s","omega_rr_rad_s","omega_fl_rad_s","omega_fr_rad_s",
        "tau_rl_Nm","tau_rr_Nm","tau_fl_Nm","tau_fr_Nm","tau_total_Nm")}
    if len(data)<10:return out
    vals=[numeric(x) for x in data[:10]]
    if not all(math.isfinite(x) for x in vals):return out
    f,dref,dact,rl,rr,_,fl,fr,torque_rear,torque_front=vals
    out.update(F_cmd_N=f,delta_ref_deg=math.degrees(dref),
        delta_actual_deg=math.degrees(dact),
        omega_rl_rad_s=rl,omega_rr_rad_s=rr,
        omega_fl_rad_s=fl,omega_fr_rad_s=fr,
        tau_rl_Nm=torque_rear,tau_rr_Nm=torque_rear,
        tau_fl_Nm=torque_front,tau_fr_Nm=torque_front,
        tau_total_Nm=2.0*(torque_rear+torque_front))
    return out

def export_csv(path,rows):
    """CSV readable by MATLAB readtable. NaNs are blanks for reproducibility."""
    path=Path(path)
    with path.open("w",newline="",encoding="utf-8-sig") as fp:
        writer=csv.DictWriter(fp,fieldnames=FIELDS,extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({
                k:("" if isinstance(row.get(k),float) and
                           not math.isfinite(row.get(k)) else row.get(k,""))
                for k in FIELDS
            })

_ALIASES={
    "t_s":("t_s","t","time","time_s","time_sec","time_seconds","seconds"),
    "x_rear_m":("x_rear_m","xp","x_p","x","x_m"),
    "y_rear_m":("y_rear_m","yp","y_p","y","y_m"),
    "v_m_s":("v_m_s","v","velocity","speed","speed_m_s"),
    "yaw_deg":("yaw_deg","heading_deg","theta_deg","heading"),
    "yaw_rad":("yaw_rad","heading_rad","theta_rad","theta"),
    "delta_actual_deg":("delta_actual_deg","steer_deg","steer_angle_deg"),
    "delta_actual_rad":("delta_actual_rad","delta_rad","psi","psi_rad","steer_rad"),
    "tau_total_Nm":("tau_total_nm","tau_nm","torque_nm","torque_total_nm","tau"),
    "F_cmd_N":("f_cmd_n","f_drive_eq_n","force_n","f_n"),
}
def load_matlab_csv(path,wheel_radius_m=0.15):
    """Import named-header CSV (MATLAB export helper or simple MATLAB table)."""
    path=Path(path)
    with path.open("r",newline="",encoding="utf-8-sig") as fp:
        reader=csv.DictReader(fp)
        if not reader.fieldnames:
            raise ValueError("CSV must contain header row; use export_ackermann_hmi_csv.m")
        names={n.strip().lower():n for n in reader.fieldnames if n}
        mapping={}
        for dest,aliases in _ALIASES.items():
            for alias in aliases:
                if alias in names:
                    mapping[dest]=names[alias]
                    break
        if "t_s" not in mapping:
            raise ValueError("MATLAB CSV must have a t_s or time column")
        raw=list(reader)
    rows=[]
    for line in raw:
        row={k:numeric(line.get(src)) for k,src in mapping.items()}
        if not math.isfinite(row.get("t_s",math.nan)):continue
        if not math.isfinite(row.get("yaw_deg",math.nan)):
            yaw=row.get("yaw_rad",math.nan)
            if math.isfinite(yaw):row["yaw_deg"]=math.degrees(yaw)
        if not math.isfinite(row.get("delta_actual_deg",math.nan)):
            delta=row.get("delta_actual_rad",math.nan)
            if math.isfinite(delta):row["delta_actual_deg"]=math.degrees(delta)
        if not math.isfinite(row.get("F_cmd_N",math.nan)):
            tau=row.get("tau_total_Nm",math.nan)
            if math.isfinite(tau):row["F_cmd_N"]=tau/wheel_radius_m
        if not math.isfinite(row.get("tau_total_Nm",math.nan)):
            f=row.get("F_cmd_N",math.nan)
            if math.isfinite(f):row["tau_total_Nm"]=f*wheel_radius_m
        rows.append(row)
    rows.sort(key=lambda r:r["t_s"])
    if len(rows)<2:raise ValueError("MATLAB CSV needs at least 2 finite rows")
    t0=rows[0]["t_s"]
    for row in rows:row["t_s"]-=t0
    # Repeated timestamps lead to zero-length interpolation intervals.
    dedup=[]
    for r in rows:
        if dedup and r["t_s"]<=dedup[-1]["t_s"]+1e-9:dedup[-1]=r
        else:dedup.append(r)
    if len(dedup)<2:raise ValueError("MATLAB CSV contains only one unique timestamp")
    return dedup

def interpolate(series,t,key,times=None):
    """Linear interpolation with NaN outside MATLAB time support.

    Pass a precomputed times array when evaluating thousands of ROS samples.
    """
    if not series or t<series[0]["t_s"] or t>series[-1]["t_s"]:
        return math.nan
    if times is None:times=[r["t_s"] for r in series]
    i=bisect.bisect_left(times,t)
    if i<len(times) and abs(times[i]-t)<1e-9:
        return numeric(series[i].get(key))
    if not 0<i<len(times):return math.nan
    a,b=series[i-1],series[i]
    va=numeric(a.get(key));vb=numeric(b.get(key))
    if not math.isfinite(va) or not math.isfinite(vb):return math.nan
    return va+(vb-va)*(t-a["t_s"])/(b["t_s"]-a["t_s"])

def yaw_error_deg(a,b):
    return math.degrees(math.atan2(
        math.sin(math.radians(a-b)),math.cos(math.radians(a-b))))

def compare_rmse(gazebo,matlab,keys=None):
    """Time-aligned RMSE; no invented zero padding or extrapolation."""
    if keys is None:keys=[k for k,_ in COMPARED]
    report={}
    times=[r["t_s"] for r in matlab]
    for key in keys:
        diffs=[]
        for row in gazebo:
            t=numeric(row.get("t_s"))
            val=numeric(row.get(key))
            if not math.isfinite(t) or not math.isfinite(val):continue
            ref=interpolate(matlab,t,key,times=times)
            if not math.isfinite(ref):continue
            delta=yaw_error_deg(val,ref) if key=="yaw_deg" else val-ref
            diffs.append(delta)
        report[key]={"n":len(diffs),
                     "rmse":math.sqrt(sum(d*d for d in diffs)/len(diffs))
                             if diffs else math.nan}
    return report


def comparison_warnings(gazebo,matlab):
    """Flag conditions that make a MATLAB/Gazebo RMSE physically misleading.

    The returned messages are diagnostics, NOT an automatic physics match
    certificate. Full equivalence also requires matching mass, tire/road
    physics, actuation mode, terrain and controller.
    """
    if not gazebo or not matlab:
        return ['Cần dữ liệu cả hai mô phỏng.']
    warnings=[]
    first_g,first_m=gazebo[0],matlab[0]
    for field,tol,label in (
        ('x_rear_m',0.25,'X ban đầu'),
        ('y_rear_m',0.25,'Y ban đầu'),
        ('v_m_s',0.15,'vận tốc ban đầu')):
        a=numeric(first_g.get(field));b=numeric(first_m.get(field))
        if math.isfinite(a) and math.isfinite(b) and abs(a-b)>tol:
            warnings.append(f'{label} khác nhau {abs(a-b):.2f} (ngưỡng {tol}).')
    a=numeric(first_g.get('yaw_deg'));b=numeric(first_m.get('yaw_deg'))
    if math.isfinite(a) and math.isfinite(b):
        d=abs(yaw_error_deg(a,b))
        if d>6.0:
            warnings.append(f'Góc yaw ban đầu lệch {d:.1f}° (ngưỡng 6°).')
    force=compare_rmse(gazebo,matlab,keys=['F_cmd_N'])['F_cmd_N']
    if force['n']>=5 and math.isfinite(force['rmse']) and force['rmse']>20.0:
        warnings.append(
            f'Lệnh lực quy đổi khác nhau: RMSE={force["rmse"]:.1f} N (>20 N).')
    return warnings


def export_comparison_csv(path,gazebo,matlab,keys=None):
    """Interpolate MATLAB at each measured Gazebo time; label every column."""
    if keys is None:keys=[k for k,_ in COMPARED]
    times=[r["t_s"] for r in matlab]
    with Path(path).open("w",newline="",encoding="utf-8-sig") as fp:
        cols=["t_s"]
        for key in keys:cols.extend(("Gazebo_"+key,"MATLAB_"+key,"error_"+key))
        writer=csv.DictWriter(fp,fieldnames=cols)
        writer.writeheader()
        for r in gazebo:
            t=numeric(r.get("t_s"))
            if not math.isfinite(t):continue
            out={"t_s":t}
            for key in keys:
                a=numeric(r.get(key));b=interpolate(matlab,t,key,times=times)
                out["Gazebo_"+key]=a
                out["MATLAB_"+key]=b
                out["error_"+key]=(yaw_error_deg(a,b) if key=="yaw_deg" else a-b) if (
                    math.isfinite(a) and math.isfinite(b)) else math.nan
            writer.writerow(out)
