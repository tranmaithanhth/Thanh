#!/usr/bin/env python3
"""Check PNG/SDF/obstacle placement independently of Gazebo itself."""
from pathlib import Path
import importlib.util
import json
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('terrain_generator',ROOT/'tools'/'generate_terrain.py')
g=importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
report=json.loads((ROOT/'export_report.json').read_text())
sdf=ET.parse(ROOT/'models'/'ackermann_matlab_terrain_v102'/'model.sdf')
world=ET.parse(ROOT/'worlds'/'ackermann_matlab_terrain.world')
model=sdf.find('model')
assert model.attrib['name']=='ackermann_matlab_terrain_v102'
assert model.findtext('pose').startswith('0 0 0 ')
hms=model.findall('.//heightmap')
assert len(hms)==2
for h in hms:
    assert h.findtext('pos') == '0 0 0'
    size=[float(i) for i in h.findtext('size').split()]
    assert abs(size[0]-size[1])<1e-8
    assert abs(size[0]-40)<1e-8
    assert h.findtext('uri').startswith('model://ackermann_matlab_terrain_v102/')

world_elem=world.find('world')
models={m.attrib['name']:m for m in world_elem.findall('model')}
assert len([key for key in models if key.startswith('obstacle_')]) ==4
world_half=report['heightmapSize'][0]/2
shift=report['matlabToGazeboZShift']
assert abs(shift)<0.2
png=np.array(Image.open(ROOT/'models'/'ackermann_matlab_terrain_v102'/'terrain_height.png'))
assert png.shape==(report["gridPixels"],report["gridPixels"])
assert png.dtype==np.uint8
# Raster interpolation check at model positions: ensure the terrain
# heights all refer to same world x,y as SDF poses.
def terrain_png_z(x,y):
    u=(x+world_half)/(2*world_half)*(png.shape[1]-1)
    v=(world_half-y)/(2*world_half)*(png.shape[0]-1)
    x0,y0=int(np.floor(u)),int(np.floor(v))
    x0=min(max(0,x0),png.shape[1]-2);y0=min(max(0,y0),png.shape[0]-2)
    du,dv=u-x0,v-y0
    vals=np.array([[png[y0,x0],png[y0,x0+1]],[png[y0+1,x0],png[y0+1,x0+1]]],dtype=float)
    intensity=(1-dv)*((1-du)*vals[0,0]+du*vals[0,1])+dv*((1-du)*vals[1,0]+du*vals[1,1])
    return float(intensity*report['heightmapSize'][2]/255)

for name,model in models.items():
    pose=np.array([float(a) for a in model.findtext('pose').split()])
    x,y,z=pose[:3]
    assert abs(x)<=world_half-2 and abs(y)<=world_half-2, (name,x,y)
    true_z=float(g.height(x,y))+shift
    q_z=terrain_png_z(x,y)
    assert abs(q_z-true_z)<.012,(name,q_z,true_z)
    if name.startswith('obstacle_'):
        assert abs(z-0.6-true_z)<.002,(name,z,true_z)
    if name in ('start_marker','goal_marker'):
        assert abs(z-0.023-true_z)<.002,(name,z,true_z)
    print(f'{name:14s} world=({x:5.2f},{y:5.2f}) terrain={q_z:.3f} m / model base aligned')

# Road elevations: uphill / plateau / downhill
profile=[float(g.height(x,0)) for x in [2.5,6.5,10.6,14.6]]
assert profile[1]>profile[0]+0.9
assert profile[2]>profile[3]+0.9
print('PASS: heightmap center & collision zero, all obstacles/markers in-bounds and Z-aligned')