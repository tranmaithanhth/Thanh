#!/usr/bin/env python3
"""Split a full xacro-expanded ackermann_vehicle URDF into physics stages.

This intentionally does not modify the original package or its xacro files.
Stages:
 A: base_footprint + chassis only
 B: chassis + 4 wheels + steering/spin joints, no gazebo_ros plugin, no sensors
 C: stage B + ackermann drive plugin
 D: stage C + LiDAR only
 E: stage C + depth camera only
 F: stage C + both sensors (original full URDF)
"""
import argparse
import sys
import xml.etree.ElementTree as ET

VALID = tuple('ABCDEF')

def subset(stage, root):
    stage = stage.upper()
    if stage not in VALID:
        raise ValueError(f'Choose stage A/B/C/D/E/F, got {stage}')
    body_only = stage == 'A'
    lidar = stage in ('D', 'F')
    camera = stage in ('E', 'F')
    drive = stage in ('C', 'D', 'E', 'F')

    def keep_link(name):
        if name in ('base_footprint', 'chassis'):
            return True
        if name.startswith('lidar'):
            return lidar
        if name.startswith('camera'):
            return camera
        return not body_only

    keep = {el.attrib.get('name') for el in root.findall('link')
            if keep_link(el.attrib.get('name', ''))}
    if 'chassis' not in keep:
        raise ValueError('Expected chassis link missing. Check input XACRO.')

    remove = []
    for el in root:
        if el.tag == 'link':
            if el.get('name') not in keep:
                remove.append(el)
        elif el.tag == 'joint':
            par, child = el.find('parent'), el.find('child')
            if par is None or child is None:
                raise ValueError(f'Joint {el.get("name")} missing parent/child')
            if par.get('link') not in keep or child.get('link') not in keep:
                remove.append(el)
        elif el.tag == 'gazebo':
            ref = el.get('reference')
            if ref:
                if ref not in keep:
                    remove.append(el)
            else:
                # Gazebo's ackermann_drive plugin is emitted as <gazebo><plugin ...>
                # Drop *all* top-level model plugins before stage C.
                if not drive:
                    remove.append(el)
        elif el.tag == 'transmission':
            jn = el.findtext('joint')
            if jn and jn not in [j.get('name') for j in root.findall('joint')]:
                remove.append(el)
    for el in remove:
        root.remove(el)

    # Ensure no robot joints reference missing links after pruning.
    links = {el.get('name') for el in root.findall('link')}
    joints = root.findall('joint')
    for j in joints:
        assert j.find('parent').get('link') in links, j.get('name')
        assert j.find('child').get('link') in links, j.get('name')

    stats = {
        'links': len(links),
        'joints': len(joints),
        'gazebo_extensions': len(root.findall('gazebo')),
        'model_plugins': sum(len(g.findall('plugin')) for g in root.findall('gazebo')),
        'sensor_blocks': sum(len(g.findall('sensor')) for g in root.findall('gazebo')),
    }
    return root, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('stage', choices=VALID)
    ap.add_argument('input_urdf')
    ap.add_argument('output_urdf')
    args = ap.parse_args()
    root = ET.parse(args.input_urdf).getroot()
    if root.tag != 'robot':
        sys.exit('Input is not a robot URDF')
    root, stats = subset(args.stage, root)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(args.output_urdf, encoding='utf-8', xml_declaration=True)
    print(f'[STAGE {args.stage}] {stats} | wrote {args.output_urdf}')

if __name__ == '__main__':
    main()