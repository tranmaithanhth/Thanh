#!/usr/bin/env python3
"""Derive test URDF stages from an already expanded ackermann_vehicle URDF.

Stages: 0 chassis; 1 +rear wheels; 2 +front steering/wheels;
        3 +Ackermann Gazebo plugin; 4 +lidar; 5 +depth camera.
Does not modify the source XACRO or the installed ROS 2 package.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

BASIC_LINKS = {"base_footprint", "chassis"}
REAR_LINKS = {"rear_left_wheel", "rear_right_wheel"}
FRONT_LINKS = {
    "front_left_steering_link", "front_right_steering_link",
    "front_left_wheel", "front_right_wheel",
}
LIDAR_LINKS = {"lidar_link"}
CAMERA_LINKS = {"camera_link", "camera_optical_link"}

BASIC_JOINTS = {"base_footprint_joint"}
REAR_JOINTS = {"rear_left_wheel_joint", "rear_right_wheel_joint"}
FRONT_JOINTS = {
    "front_left_steer_joint", "front_right_steer_joint",
    "front_left_wheel_joint", "front_right_wheel_joint",
}
LIDAR_JOINTS = {"lidar_joint"}
CAMERA_JOINTS = {"camera_joint", "camera_optical_joint"}


def make_stage(source: Path, target: Path, stage: int, safe_friction: bool, drive_profile: str = 'original') -> dict:
    if stage not in range(6):
        raise ValueError("stage must be 0, 1, 2, 3, 4 or 5")
    if drive_profile not in ('original', 'no_motors', 'gentle'):
        raise ValueError('drive_profile must be original, no_motors or gentle')
    if stage < 3 and drive_profile != 'original':
        raise ValueError('drive PID profiles require stage >= 3')
    tree = ET.parse(source)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError("Source must be a URDF robot root")

    allowed_links = set(BASIC_LINKS)
    allowed_joints = set(BASIC_JOINTS)
    if stage >= 1:
        allowed_links.update(REAR_LINKS)
        allowed_joints.update(REAR_JOINTS)
    if stage >= 2:
        allowed_links.update(FRONT_LINKS)
        allowed_joints.update(FRONT_JOINTS)
    if stage >= 4:
        allowed_links.update(LIDAR_LINKS)
        allowed_joints.update(LIDAR_JOINTS)
    if stage >= 5:
        allowed_links.update(CAMERA_LINKS)
        allowed_joints.update(CAMERA_JOINTS)

    # Fail early if the user's own XACRO uses different names. Better than
    # silently producing a body without the intended wheels or controller.
    have_links = {element.get("name") for element in root.findall("link")}
    have_joints = {element.get("name") for element in root.findall("joint")}
    required_missing = (allowed_links - have_links) | (allowed_joints - have_joints)
    if required_missing:
        raise ValueError("Expected links/joints not found in XACRO: " + ", ".join(sorted(required_missing)))

    for element in list(root):
        if element.tag == "link" and element.get("name") not in allowed_links:
            root.remove(element)
        elif element.tag == "joint" and element.get("name") not in allowed_joints:
            root.remove(element)
        elif element.tag == "gazebo":
            reference = element.get("reference")
            if reference is not None:
                if reference not in allowed_links:
                    root.remove(element)
                    continue
                if safe_friction and "wheel" in reference:
                    for key in ("mu1", "mu2"):
                        node = element.find(key)
                        if node is not None:
                            node.text = "1.0"
                    # The original wheel macros use a fixed fdir1 vector.
                    # Remove it for this isolated contact-solver diagnostic.
                    for direction in element.findall("fdir1"):
                        element.remove(direction)
            else:
                # Only retain the drive plugin at stage 3 and beyond.
                for plugin in list(element.findall("plugin")):
                    if stage < 3 or plugin.get("name") != "ackermann_drive":
                        element.remove(plugin)
                        continue
                    if drive_profile != "original":
                        # Diagnostic variants change only command-producing PIDs:
                        # keep gazebo_ros_ackermann_drive plugin and world odometry.
                        # no_motors: motors apply zero effort even if PID.Update runs.
                        # gentle: much smaller P gains, no D kick, to test stability.
                        gains = ({
                            "linear_velocity_pid_gain": "0 0 0",
                            "left_steering_pid_gain": "0 0 0",
                            "right_steering_pid_gain": "0 0 0",
                        } if drive_profile == "no_motors" else {
                            "linear_velocity_pid_gain": "2 0 0",
                            "left_steering_pid_gain": "20 0 0",
                            "right_steering_pid_gain": "20 0 0",
                        })
                        for tag, values in gains.items():
                            target_gain = plugin.find(tag)
                            if target_gain is None:
                                raise ValueError(f"Missing {tag} in ackermann_drive")
                            target_gain.text = values
                if len(element) == 0:
                    root.remove(element)

    actual_links = {element.get("name") for element in root.findall("link")}
    actual_joints = {element.get("name") for element in root.findall("joint")}
    assert actual_links == allowed_links, (actual_links, allowed_links)
    assert actual_joints == allowed_joints, (actual_joints, allowed_joints)
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {joint.get('name')} lacks parent or child")
        if parent.get("link") not in actual_links or child.get("link") not in actual_links:
            raise ValueError(f"Joint {joint.get('name')} references removed link")

    plugin_names = [n.get("name") for n in root.findall(".//plugin")]
    if stage < 3 and plugin_names:
        raise ValueError("Gazebo plugin accidentally enabled in stage 0, 1 or 2")
    if stage >= 3 and "ackermann_drive" not in plugin_names:
        raise ValueError("Ackermann plugin missing from expanded XACRO")
    if stage >= 4 and not any("laser" in (n or "") for n in plugin_names):
        raise ValueError("LiDAR plugin missing from expanded XACRO")
    if stage >= 5 and not any("camera" in (n or "") for n in plugin_names):
        raise ValueError("Camera plugin missing from expanded XACRO")

    target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(target, encoding="utf-8", xml_declaration=False)
    return {"stage": stage, "links": len(actual_links), "joints": len(actual_joints), "plugins": plugin_names,
            "safe_friction": safe_friction, "drive_profile": drive_profile}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="Expanded URDF generated by xacro")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", type=int, required=True, choices=range(6))
    parser.add_argument("--safe-friction", action="store_true",
                        help="Diagnostic only: lower wheel mu1/mu2 to 1.0, drop wheel fdir1")
    parser.add_argument("--drive-profile", choices=['original', 'no_motors', 'gentle'], default='original',
                        help='Ackermann Drive PID A/B tests; no motors keeps odometry, gentle lowers PID gains')
    args = parser.parse_args()
    info = make_stage(args.source, args.output, args.stage, args.safe_friction, args.drive_profile)
    print("Generated staged URDF:", args.output)
    print("Summary:", info)


if __name__ == "__main__":
    main()