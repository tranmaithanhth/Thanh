#!/usr/bin/env python3
"""Convert staged-3 URDF to MATLAB-scaled force-driven robot (no sensors)."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def patch(source: Path, destination: Path, params: dict):
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
    mass.set('value', str(params['chassis_mass_kg']))
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
        'max_steer': params['max_steer_rad'],
        'max_steer_rate': params['max_steer_rate_rad_s'],
        'steering_kp': 25.0, 'steering_kd': 3.0,
        'max_steer_torque': 25.0, 'command_timeout': 0.6,
    }
    for name, value in values.items():
        ET.SubElement(plugin,name).text = str(value)
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
    print(f'[MODEL] MATLAB mass-aligned: {mass:.3f}kg, wheel J_spin=0.03kg.m2, force/steering ROS plugin, mu=1.1 assumed')

if __name__=='__main__':
    main()