#!/usr/bin/env python3
"""Offline geometry and XML validation for HMI Gazebo scene editing.

This operator tool edits a *simulation scene*; it never inserts object
coordinates into mission.json or the Ackermann navigator's initial A* map.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET

ROBOT_NAME="ackermann_force_nearby_v104"
OBSTACLE_PREFIX="hmi_obstacle_"
WORLD_EDIT_LIMIT=18.0
OBSTACLE_PREFIXES=("obstacle_",OBSTACLE_PREFIX,"lidar_test_box","test_unknown_box")


def is_obstacle_name(name):
    return (isinstance(name,str) and name.startswith(OBSTACLE_PREFIXES)
            and name!=ROBOT_NAME)


def removable_models(names):
    return sorted(n for n in names if is_obstacle_name(n))


def unique_obstacle_name(names,bounding_radius_m=0.75):
    # Encode ONLY an approximate collision radius in the model name, not XY.
    # NearbyScene reads it when the object enters its 4m observation range.
    bound=float(bounding_radius_m)
    if not math.isfinite(bound) or not (0.15<=bound<=5.):
        raise ValueError("Invalid bounding radius")
    cm=math.ceil(bound*100)
    used=set(names)
    used_serials={
        int(match.group(1)) for name in used
        if (match:=re.fullmatch(
            r"hmi_obstacle_(\\d{3,4})(?:_r\\d{3,4})?",name))
    }
    for i in range(1,10000):
        if i in used_serials:continue
        name=f"{OBSTACLE_PREFIX}{i:03d}_r{cm:03d}"
        if name not in used:
            return name
    raise RuntimeError("Too many HMI obstacles")


def validate_xy(x,y,bounds=None,margin=0.0):
    x=float(x);y=float(y);margin=float(margin)
    if not all(math.isfinite(n) for n in (x,y,margin)) or margin<0:
        raise ValueError("Invalid coordinates")
    if bounds is None:
        bounds=(-WORLD_EDIT_LIMIT,WORLD_EDIT_LIMIT,
                -WORLD_EDIT_LIMIT,WORLD_EDIT_LIMIT)
    xmin,xmax,ymin,ymax=map(float,bounds)
    if not(xmin+margin<x<xmax-margin and
           ymin+margin<y<ymax-margin):
        raise ValueError("Position is outside editable simulation area")
    return x,y


def validate_obstacle(shape,size_m,height_m):
    if shape not in ("cylinder","box"):
        raise ValueError("Obstacle must be cylinder or box")
    r=float(size_m);h=float(height_m)
    if not all(math.isfinite(v) for v in (r,h)):
        raise ValueError("Obstacle dimensions must be finite")
    if not (0.15<=r<=2.5 and 0.15<=h<=3.0):
        raise ValueError("Obstacle size/height out of simulation limits")
    return r,h


def obstacle_sdf(name,shape,size_m,height_m):
    """Static collision+visual Gazebo SDF, never a decorative marker."""
    if not (isinstance(name,str) and re.fullmatch(r"hmi_obstacle_\d{3,4}(?:_r\d{3,4})?",name)):
        raise ValueError("Invalid HMI obstacle name")
    r,h=validate_obstacle(shape,size_m,height_m)
    root=ET.Element("sdf",{"version":"1.6"})
    model=ET.SubElement(root,"model",{"name":name})
    ET.SubElement(model,"static").text="true"
    link=ET.SubElement(model,"link",{"name":"body"})
    for kind,attr in (("collision","solid"),("visual","visible")):
        el=ET.SubElement(link,kind,{"name":attr})
        geometry=ET.SubElement(el,"geometry")
        if shape=="cylinder":
            cylinder=ET.SubElement(geometry,"cylinder")
            ET.SubElement(cylinder,"radius").text=f"{r:.6f}"
            ET.SubElement(cylinder,"length").text=f"{h:.6f}"
        else:
            box=ET.SubElement(geometry,"box")
            ET.SubElement(box,"size").text=f"{2*r:.6f} {2*r:.6f} {h:.6f}"
        if kind=="visual":
            mat=ET.SubElement(el,"material")
            ET.SubElement(mat,"ambient").text="0.96 0.52 0.12 1"
            ET.SubElement(mat,"diffuse").text="0.96 0.52 0.12 1"
    return ET.tostring(root,encoding="unicode")


def ground_center_height(x,y,object_height,terrain_fn,shift_m):
    z=float(terrain_fn(float(x),float(y))[0])+float(shift_m)
    h=float(object_height)
    if not math.isfinite(z) or not math.isfinite(h) or h<=0:
        raise ValueError("Invalid terrain or obstacle height")
    return z+0.5*h+0.015


def conflicts_with_robot(x,y,obstacle_size,robot_pose,clearance=0.45):
    if robot_pose is None:
        return False
    rx,ry=robot_pose
    # Approximate 0.85 m body bounding radius, conservative before spawn.
    return math.hypot(float(x)-rx,float(y)-ry) < (
        0.85+math.sqrt(2)*float(obstacle_size)+float(clearance))


def conflicts_with_obstacles(x,y,robot_radius,model_positions,
                             candidate_radius=0.75):
    """Prevent a robot from being respawned inside known scenario objects."""
    for name,ox,oy in model_positions:
        if not is_obstacle_name(name):continue
        if math.hypot(float(x)-ox,float(y)-oy) <= robot_radius+candidate_radius:
            return True
    return False
