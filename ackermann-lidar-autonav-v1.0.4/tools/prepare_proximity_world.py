#!/usr/bin/env python3
"""Add Gazebo Classic ROS 2 model-state world plugin to a throwaway SDF copy.

Never edit v1.0.2/v1.0.3 terrain files in place, preserving MATLAB world and
tire/force configuration. Requires libgazebo_ros_state.so in ROS Humble.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_world(source, dest):
    root=ET.parse(source).getroot()
    world=root.find('world')
    if world is None:
        raise ValueError('The input SDF has no <world>')
    if world.find("plugin[@name='ackermann_proximity_world_state']") is None:
        plugin=ET.SubElement(world,'plugin',{
            'name':'ackermann_proximity_world_state',
            'filename':'libgazebo_ros_state.so'})
        ros=ET.SubElement(plugin,'ros')
        ET.SubElement(ros,'namespace').text='/gazebo'
        ET.SubElement(ros,'remapping').text='model_states:=/gazebo/model_states'
        ET.SubElement(plugin,'update_rate').text='10'
    dest=Path(dest)
    dest.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(root,space='  ')
    ET.ElementTree(root).write(dest,encoding='utf-8',xml_declaration=True)
    return dest


if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('--input',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    print('[PROXIMITY WORLD] '+str(prepare_world(args.input,args.output)))
