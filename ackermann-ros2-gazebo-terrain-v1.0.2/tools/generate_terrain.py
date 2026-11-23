#!/usr/bin/env python3
"""Generate Gazebo Classic terrain accurately aligned with MATLAB world XY.

The critical v1.0.2 fix is *not moving a heightmap model*. Gazebo Classic's
heightmap visual / collision have a history of differing when the model is
translated. Instead the terrain is sampled symmetrically about world (0,0)
and the model/heightmap geometry each have identity transforms. Obstacles,
start and goal retain their MATLAB XY world coordinates. Only a small,
constant positive Z translation of the height function is needed to store
all heights in unsigned PNG; all Gazebo entities get the same translation.

Run: python3 tools/generate_terrain.py
Requires numpy + Pillow only when REGENERATING, not to use committed world.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "ackermann_matlab_terrain_v102"
WORLDS = ROOT / "worlds"
CFG = json.loads((ROOT / "terrain_params.json").read_text(encoding="utf-8"))


def smoothstep(x, x0, x1):
    if not x1 > x0:
        raise ValueError("Ramp end must exceed start")
    t = np.clip((np.asarray(x) - x0) / (x1 - x0), 0, 1)
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def height(x, y):
    """MATLAB terrain3d_height.m from Terrain3D v1.0.4 (same formula)."""
    x, y = np.asarray(x), np.asarray(y)
    a = smoothstep(x, *CFG["terrainRiseX"])
    b = smoothstep(x, *CFG["terrainFallX"])
    return (CFG["terrainHeight"] * (a - b)
            + CFG["terrainCamber"] * np.sin(0.55 * x) * np.tanh(y / 2.5)
            + CFG["terrainRipple"] * np.sin(0.45 * x + 0.5 * y))


def tag(parent, tagname, text=None, **attrs):
    e = ET.SubElement(parent, tagname, attrs)
    if text is not None:
        e.text = str(text)
    return e


def format_xml(root):
    ET.indent(root, space="  ")
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding="unicode") + '\n'


def make_heightmap(parent, uri, side, zrange):
    geometry = tag(parent, "geometry")
    hm = tag(geometry, "heightmap")
    tag(hm, "uri", uri)
    tag(hm, "size", f"{side:.9f} {side:.9f} {zrange:.9f}")
    tag(hm, "pos", "0 0 0")
    return hm


def make_model(side, zrange):
    sdf = ET.Element("sdf", version="1.6")
    model = tag(sdf, "model", name="ackermann_matlab_terrain_v102")
    tag(model, "static", "true")
    # DO NOT translate the model: this caused visual/collision XY mismatches.
    tag(model, "pose", "0 0 0 0 0 0")
    link = tag(model, "link", name="terrain_link")
    uri = "model://ackermann_matlab_terrain_v102/terrain_height.png"
    collision = tag(link, "collision", name="terrain_collision")
    make_heightmap(collision, uri, side, zrange)
    surface = tag(collision, "surface")
    friction = tag(surface, "friction")
    ode = tag(friction, "ode")
    for name, val in [("mu", "1.1"), ("mu2", "1.1"), ("slip1", "0"), ("slip2", "0")]:
        tag(ode, name, val)
    visual = tag(link, "visual", name="terrain_visual")
    hm = make_heightmap(visual, uri, side, zrange)
    texture = tag(hm, "texture")
    tag(texture, "diffuse", "file://media/materials/textures/grass_diffusespecular.png")
    tag(texture, "normal", "file://media/materials/textures/flat_normal.png")
    tag(texture, "size", "3")
    return format_xml(sdf)


def cylinder_model(world, name, x, y, zcenter, radius, length, color, collision=True):
    model = tag(world, "model", name=name)
    tag(model, "static", "true")
    tag(model, "pose", f"{x:.6f} {y:.6f} {zcenter:.6f} 0 0 0")
    link = tag(model, "link", name="body")
    for kind in (["collision", "visual"] if collision else ["visual"]):
        elem = tag(link, kind, name=kind)
        geom = tag(elem, "geometry")
        cyl = tag(geom, "cylinder")
        tag(cyl, "radius", f"{radius:.6f}")
        tag(cyl, "length", f"{length:.6f}")
        if kind == "visual":
            mat = tag(elem, "material")
            tag(mat, "ambient", color)
            tag(mat, "diffuse", color)


def make_world(height_shift):
    sdf = ET.Element("sdf", version="1.6")
    world = tag(sdf, "world", name="ackermann_matlab_terrain_v102")
    tag(world, "gravity", "0 0 -9.81")
    physics = tag(world, "physics", type="ode")
    tag(physics, "max_step_size", "0.001")
    tag(physics, "real_time_update_rate", "1000")
    include = tag(world, "include")
    tag(include, "uri", "model://sun")
    # Do NOT include ground_plane; its collisions would mask the heightmap.
    include = tag(world, "include")
    tag(include, "uri", "model://ackermann_matlab_terrain_v102")

    obstacles = np.asarray(CFG["obstacles"], dtype=float).reshape(-1, 3)
    for i, (x,y,radius) in enumerate(obstacles, 1):
        z = float(height(x,y)) + height_shift
        cylinder_model(world, f"obstacle_{i:02d}", x,y,z + 0.6,
                       radius, 1.2, "0.82 0.21 0.16 1", True)
    # Low, noncolliding markers. The start is blue and the goal green.
    for label, xy, rgba in [
        ("start_marker", CFG["start"], "0.05 0.30 0.95 1"),
        ("goal_marker", CFG["goal"], "0.05 0.80 0.22 1")]:
        x,y = [float(a) for a in xy]
        z = float(height(x,y)) + height_shift
        cylinder_model(world, label, x,y,z+0.023,0.26,0.046,rgba,False)

    gui = tag(world,"gui",fullscreen="0")
    camera = tag(gui,"camera",name="camera")
    # Camera looks at the working corridor x=0..16 instead of the entire
    # (larger) square map required for reliable world-origin alignment.
    tag(camera,"pose", "8 -17 15 0 0.60 1.57")
    tag(camera,"view_controller", "orbit")
    return format_xml(sdf)


def export():
    n = int(CFG["gridPixels"])
    if n < 33 or ((n-1) & (n-2)) != 0:
        raise ValueError("gridPixels must be 2^k+1, e.g. 257, 513, 1025")
    half = float(CFG["worldHalfExtent"])
    margin = float(CFG["edgeMargin"])
    pts = [CFG["start"], CFG["goal"]]
    for x,y,r in CFG["obstacles"]:
        pts.extend([(x+r,y+r),(x-r,y-r)])
    maxcoord = max(abs(float(a)) for xy in pts for a in xy)
    half = max(half, math.ceil((maxcoord + margin)*2)/2)
    side = 2.0*half
    grid = np.linspace(-half, half, n)
    xx, yy = np.meshgrid(grid, grid)
    true_h = height(xx, yy)
    hmin = float(np.min(true_h))
    # Geometry transform remains exactly zero: all coordinates retain
    # MATLAB XY values, while Gazebo Z values are shifted by a constant.
    shift = -min(0.0, hmin)
    new_h = true_h + shift
    zrange = float(np.max(new_h))
    if zrange <= 0: raise ValueError("Height range must be positive")
    uint = np.uint8(np.rint(np.clip(new_h/zrange,0,1)*255))
    if CFG.get("flip_y", True):
        uint = np.flipud(uint)
    MODEL.mkdir(parents=True, exist_ok=True)
    WORLDS.mkdir(parents=True, exist_ok=True)
    Image.fromarray(uint, mode="L").save(MODEL/"terrain_height.png")
    (MODEL/"model.sdf").write_text(make_model(side, zrange), encoding="utf-8")
    config = '<?xml version="1.0"?>\n<model>\n  <name>ackermann_matlab_terrain_v102</name>\n  <version>1.0.2</version>\n  <sdf version="1.6">model.sdf</sdf>\n  <description>World-origin terrain for ROS2 Humble / Gazebo Classic</description>\n</model>\n'
    (MODEL/"model.config").write_text(config, encoding="utf-8")
    (WORLDS/"ackermann_matlab_terrain.world").write_text(make_world(shift),encoding="utf-8")
    decoded = uint[::-1] if CFG.get("flip_y",True) else uint
    restored = decoded.astype(float)/255*zrange
    maxerror = float(np.max(np.abs(new_h-restored)))
    report = dict(
        version="1.0.2", gridPixels=n,
        worldBounds=[-half,half,-half,half],
        worldOrigin=[0,0,0],
        heightmapSize=[side,side,zrange],
        matlabToGazeboZShift=shift,
        imageFlipY=bool(CFG.get("flip_y", True)),
        imageQuantizationMaxErrorM=maxerror,
        obstacleCount=len(CFG["obstacles"]),
        goal=CFG["goal"],
        explanation="Gazebo Z = MATLAB terrain3d_height(x,y) + matlabToGazeboZShift",
    )
    (ROOT/"export_report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    return report


if __name__ == "__main__":
    print(json.dumps(export(),indent=2,ensure_ascii=False))