#!/usr/bin/env python3
"""Export the exact height-function *form* from MATLAB Terrain3D v1.0.4
 to a Gazebo Classic 11 heightmap + SDF model + obstacles.
 Dependencies: numpy, pillow. The generated heightmap PNG and SDF are already
 included in the distribution; rerun only after changing parameters.
"""
from pathlib import Path
import json
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "models" / "ackermann_matlab_terrain"
WORLD_DIR = ROOT / "worlds"
CFG = json.loads((ROOT / "terrain_params.json").read_text(encoding="utf-8"))


def smooth_ramp(x, x0, x1):
    t = np.clip((x-x0)/(x1-x0), 0.0, 1.0)
    return 10*t**3 - 15*t**4 + 6*t**5


def height(x,y):
    # Exact formula as MATLAB terrain3d_height.m (Terrain3D v1.0.4).
    a=smooth_ramp(x, *CFG["terrainRiseX"])
    b=smooth_ramp(x, *CFG["terrainFallX"])
    return (CFG["terrainHeight"]*(a-b)
        +CFG["terrainCamber"]*np.sin(0.55*x)*np.tanh(y/2.5)
        +CFG["terrainRipple"]*np.sin(0.45*x + 0.5*y))


def export():
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    xmin,xmax,ymin,ymax = CFG["terrainBounds"]
    # 257=2**8+1 (Gazebo Classic heightmap requirement)
    n=257
    xs=np.linspace(xmin,xmax,n)
    ys=np.linspace(ymin,ymax,n)
    xx,yy=np.meshgrid(xs,ys)
    elev=height(xx,yy)
    zmin=float(elev.min()); zmax=float(elev.max())
    zscale=zmax-zmin
    # PNG pixel row 0 is at +Y in conventional top-down representation.
    # Gazebo can use different Y conventions depending on resource backend;
    # if mirrored, toggle flip_y in terrain_params.json and regenerate.
    gray=np.uint8(np.rint(255*np.clip((elev-zmin)/zscale,0,1)))
    if CFG.get("flip_y",True):
        gray=np.flipud(gray)
    png=MODEL_DIR / 'terrain_height.png'
    Image.fromarray(gray,mode="L").save(png)

    model_sdf=f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="ackermann_matlab_terrain">
    <static>true</static>
    <pose>{(xmin+xmax)/2:.9f} {(ymin+ymax)/2:.9f} {zmin:.9f} 0 0 0</pose>
    <link name="terrain_link">
      <collision name="terrain_collision">
        <geometry>
          <heightmap>
            <uri>model://ackermann_matlab_terrain/terrain_height.png</uri>
            <size>{xmax-xmin:.9f} {ymax-ymin:.9f} {zscale:.9f}</size>
            <pos>0 0 0</pos>
          </heightmap>
        </geometry>
        <surface>
          <friction><ode><mu>1.2</mu><mu2>1.2</mu2><slip1>0</slip1><slip2>0</slip2></ode></friction>
        </surface>
      </collision>
      <visual name="terrain_visual">
        <geometry>
          <heightmap>
            <uri>model://ackermann_matlab_terrain/terrain_height.png</uri>
            <size>{xmax-xmin:.9f} {ymax-ymin:.9f} {zscale:.9f}</size>
            <pos>0 0 0</pos>
            <texture>
              <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>
              <normal>file://media/materials/textures/flat_normal.png</normal>
              <size>4</size>
            </texture>
          </heightmap>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
'''
    (MODEL_DIR/'model.sdf').write_text(model_sdf, encoding='utf-8')
    (MODEL_DIR/'model.config').write_text('''<?xml version="1.0"?>
<model><name>ackermann_matlab_terrain</name><version>1.0</version>
<sdf version="1.6">model.sdf</sdf>
<description>Terrain generated from the height formula of Ackermann Terrain3D v1.0.4.</description>
</model>
''',encoding='utf-8')

    obs_xml=[]
    for i,(x,y,r) in enumerate(CFG['obstacles'],1):
        z=float(height(float(x),float(y)))
        hobs=1.2
        obs_xml.append(f'''  <model name="obstacle_{i:02d}">
    <static>true</static>
    <pose>{x:.6f} {y:.6f} {z+hobs/2:.6f} 0 0 0</pose>
    <link name="body">
      <collision name="collision"><geometry><cylinder><radius>{r:.6f}</radius><length>{hobs:.6f}</length></cylinder></geometry></collision>
      <visual name="visual"><geometry><cylinder><radius>{r:.6f}</radius><length>{hobs:.6f}</length></cylinder></geometry>
        <material><ambient>0.85 0.24 0.18 1</ambient><diffuse>0.85 0.24 0.18 1</diffuse></material>
      </visual>
    </link>
  </model>''')
    goal_x,goal_y=CFG['goal']
    goal_h=float(height(goal_x,goal_y))
    obstacles='\n'.join(obs_xml)
    world=f'''<?xml version="1.0"?>
<sdf version="1.6">
 <world name="ackermann_matlab_terrain">
  <gravity>0 0 -9.81</gravity>
  <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
  <include><uri>model://sun</uri></include>
  <!-- No ground_plane: it would collide with the shaped heightmap. -->
  <include><uri>model://ackermann_matlab_terrain</uri></include>
{obstacles}
  <model name="goal_marker">
   <static>true</static><pose>{goal_x:.6f} {goal_y:.6f} {goal_h+0.035:.6f} 0 0 0</pose>
   <link name="marker_link">
    <!-- Visual-only marker: cannot prevent robot reaching the goal. -->
    <visual name="visual"><geometry><cylinder><radius>0.22</radius><length>0.04</length></cylinder></geometry>
     <material><ambient>0.05 0.7 0.15 1</ambient><diffuse>0.05 0.7 0.15 1</diffuse></material>
    </visual>
   </link>
  </model>
  <gui fullscreen="0"><camera name="camera"><pose>9 -13 13 0 0.55 1.57</pose><view_controller>orbit</view_controller></camera></gui>
 </world>
</sdf>
'''
    (WORLD_DIR/'ackermann_matlab_terrain.world').write_text(world,encoding='utf-8')

    # Compare source smooth terrain with nearest quantized heightmap cells.
    recon=zmin+gray.astype(float)*zscale/255
    recon=np.flipud(recon) if CFG.get('flip_y', True) else recon
    quant_err=float(np.max(np.abs(recon-elev)))
    report={"matlab_height_function":"terrain3d_height.m v1.0.4",
      "grid_pixels":[n,n],"terrain_bounds_m":[xmin,xmax,ymin,ymax],
      "height_min_m":zmin,"height_max_m":zmax,"height_range_m":zscale,
      "png_quantization_max_error_m":quant_err,
      "slope_up_m":CFG['terrainRiseX'],"slope_down_m":CFG['terrainFallX'],
      "obstacle_count":len(CFG['obstacles']),"heightmap_image":"models/ackermann_matlab_terrain/terrain_height.png"}
    (ROOT/'export_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Exported Gazebo Classic world. Quantization max error:',quant_err,'m')
    return report

if __name__=='__main__':
    export()
