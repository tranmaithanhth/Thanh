#!/usr/bin/env python3
"""Patch stage-3 Ackermann URDF with force plugin + lightweight Gazebo Classic ray LiDAR."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def patch(source: Path, destination: Path, params: dict):
    max_force = float(params['max_drive_force_N'])
    radius = float(params['radius_m'])
    enhanced_torque = float(params['enhanced_max_total_drive_torque_Nm'])
    mode = params.get('drive_mode', 'rear')
    if max_force <= 0 or radius <= 0 or abs(max_force * radius - enhanced_torque) > 1e-4:
        raise ValueError('max_drive_force_N * radius_m must equal enhanced_max_total_drive_torque_Nm')
    if mode not in ('rear','all'):
        raise ValueError('drive_mode must be rear or all')
    tree = ET.parse(source)
    root = tree.getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    required = ['chassis', 'rear_left_wheel', 'rear_right_wheel',
                'front_left_wheel', 'front_right_wheel',
                'front_left_steering_link', 'front_right_steering_link']
    if any(n not in links for n in required):
        raise RuntimeError('Expanded URDF lacks required wheel/steering links: ' + str([n for n in required if n not in links]))
    def inertia_link(name):
        inertial = links[name].find('inertial')
        if inertial is None or inertial.find('mass') is None or inertial.find('inertia') is None:
            raise RuntimeError('Missing inertia in '+name)
        return inertial.find('mass'), inertial.find('inertia')
    mass, inertia = inertia_link('chassis')
    # Add only a tiny link inertia so the complete URDF still sums to 20 kg.
    lidar_mass=0.001
    mass.set('value', str(float(params['chassis_mass_kg'])-lidar_mass))
    # These principal inertias reference the MATLAB body values; all-link composite differs.
    inertia.set('ixx', str(params['body_roll_inertia_reference_kgm2']))
    inertia.set('iyy', str(params['body_pitch_inertia_reference_kgm2']))
    inertia.set('izz', str(params['body_yaw_inertia_reference_kgm2']))
    inertia.set('ixy', '0'); inertia.set('ixz', '0'); inertia.set('iyz', '0')
    # The inertial origin has a +pi/2 roll rotation; local z corresponds to wheel axle.
    for name in ['rear_left_wheel', 'rear_right_wheel', 'front_left_wheel', 'front_right_wheel']:
        mass, inertia = inertia_link(name)
        mass.set('value', str(params['wheel_mass_kg_each']))
        inertia.set('ixx', '0.018')
        inertia.set('iyy', '0.018')
        inertia.set('izz', str(params['wheel_inertia_spin_kgm2']))
        inertia.set('ixy', '0'); inertia.set('ixz', '0'); inertia.set('iyz', '0')
    for name in ['front_left_steering_link', 'front_right_steering_link']:
        mass, _ = inertia_link(name)
        mass.set('value', str(params['steering_link_mass_kg_each']))
    mu = str(params['gazebo_tire_coulomb_mu_assumed'])
    for gz in root.findall('gazebo'):
        ref = gz.get('reference')
        if ref and 'wheel' in ref:
            for key in ('mu1', 'mu2'):
                el = gz.find(key)
                if el is None:
                    el = ET.SubElement(gz, key)
                el.text = mu
            # A fixed non-contact tangent axis (0 0 1) can cause unexpected friction.
            for el in gz.findall('fdir1'):
                gz.remove(el)
        if ref is None:
            for plg in list(gz.findall('plugin')):
                gz.remove(plg)
    gz = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gz, 'plugin', {'name':'ackermann_force_drive','filename':'libackermann_force_drive.so'})
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/ackermann_vehicle'
    values = {
        'wheel_radius': params['radius_m'],
        'wheelbase': params['wheelbase_m'],
        'track_width': params['track_m'],
        'max_force': params['max_drive_force_N'],
        'drive_mode': mode,
        'max_steer': params['max_steer_rad'],
        'max_steer_rate': params['max_steer_rate_rad_s'],
        'steering_kp': 25.0, 'steering_kd': 3.0,
        'max_steer_torque': 25.0, 'command_timeout': 0.6,
    }
    for name, value in values.items():
        ET.SubElement(plugin,name).text = str(value)
    # Fixed LiDAR link on chassis: no collision geometry, so it cannot
    # affect wheel contact/traction. The attached Gazebo Classic ray sensor
    # outputs ROS 2 LaserScan from the real sensor frame, not chassis origin.
    if root.find("link[@name='lidar_link']") is not None:
        raise RuntimeError('lidar_link already exists; do not add duplicate sensor')
    ll=ET.SubElement(root,'link',{'name':'lidar_link'})
    inert=ET.SubElement(ll,'inertial')
    ET.SubElement(inert,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
    ET.SubElement(inert,'mass',{'value':str(lidar_mass)})
    ET.SubElement(inert,'inertia',{'ixx':'0.00001','iyy':'0.00001','izz':'0.00001',
                                   'ixy':'0','ixz':'0','iyz':'0'})
    lj=ET.SubElement(root,'joint',{'name':'lidar_fixed_joint','type':'fixed'})
    ET.SubElement(lj,'parent',{'link':'chassis'})
    ET.SubElement(lj,'child',{'link':'lidar_link'})
    ET.SubElement(lj,'origin',{'xyz':'0.20 0 0.33','rpy':'0 0 0'})
    gz_lidar=ET.SubElement(root,'gazebo',{'reference':'lidar_link'})
    sensor=ET.SubElement(gz_lidar,'sensor',{'name':'lidar_2d','type':'ray'})
    ET.SubElement(sensor,'always_on').text='true'
    ET.SubElement(sensor,'visualize').text='false'
    ET.SubElement(sensor,'update_rate').text='5'
    ray=ET.SubElement(sensor,'ray')
    scan=ET.SubElement(ray,'scan')
    horizontal=ET.SubElement(scan,'horizontal')
    ET.SubElement(horizontal,'samples').text='91'
    ET.SubElement(horizontal,'resolution').text='1'
    ET.SubElement(horizontal,'min_angle').text='-3.14159265'
    ET.SubElement(horizontal,'max_angle').text='3.14159265'
    ran=ET.SubElement(ray,'range')
    ET.SubElement(ran,'min').text='0.15'
    ET.SubElement(ran,'max').text='8.0'
    ET.SubElement(ran,'resolution').text='0.03'
    noise=ET.SubElement(ray,'noise')
    ET.SubElement(noise,'type').text='gaussian'
    ET.SubElement(noise,'mean').text='0'
    ET.SubElement(noise,'stddev').text='0.008'
    sensor_plugin=ET.SubElement(sensor,'plugin',{
        'name':'gazebo_ros_ray_sensor_lidar_v104',
        'filename':'libgazebo_ros_ray_sensor.so'})
    sensor_ros=ET.SubElement(sensor_plugin,'ros')
    ET.SubElement(sensor_ros,'namespace').text='/'
    ET.SubElement(sensor_ros,'remapping').text='~/out:=/scan'
    ET.SubElement(sensor_plugin,'output_type').text='sensor_msgs/LaserScan'
    ET.SubElement(sensor_plugin,'frame_name').text='lidar_link'
    if root.findall('.//plugin[@name="ackermann_drive"]'):
        raise AssertionError('Legacy velocity plugin still active')
    count_mass = sum(float(l.find('inertial/mass').get('value')) for l in root.findall('link')
                     if l.find('inertial/mass') is not None)
    if abs(count_mass - params['total_mass_kg']) > 1e-6:
        raise AssertionError(f'Aggregate URDF masses {count_mass:.6f} != {params["total_mass_kg"]}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='utf-8', xml_declaration=False)
    return count_mass


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--input',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--params',type=Path,required=True)
    args=ap.parse_args()
    mass=patch(args.input,args.output,json.loads(args.params.read_text(encoding='utf-8')))
    params = json.loads(args.params.read_text(encoding='utf-8'))
    print(f'[MODEL V1.0.4] mass={mass:.3f}kg, AWD force_cap={params["max_drive_force_N"]:.1f}N, LiDAR=91 rays @5Hz, 360deg, /scan')

if __name__=='__main__':
    main()