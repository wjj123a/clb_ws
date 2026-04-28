#!/usr/bin/env python3

import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import rospkg
import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler


def build_pose(x, y, z, yaw):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def prepare_model_xml(model_xml, strip_depth_camera=False, disable_laser_visualization=False):
    if not strip_depth_camera and not disable_laser_visualization:
        return model_xml

    root = ET.fromstring(model_xml)

    for gazebo in list(root.findall("gazebo")):
        reference = gazebo.get("reference", "")

        if strip_depth_camera and reference == "camera_link":
            root.remove(gazebo)
            continue

        if disable_laser_visualization and reference == "laser_link":
            for sensor in gazebo.findall("sensor"):
                visualize = sensor.find("visualize")
                if visualize is not None:
                    visualize.text = "false"

    return ET.tostring(root, encoding="unicode")


def replace_model_uris(sdf_xml):
    try:
        package_path = rospkg.RosPack().get_path("r1")
    except rospkg.ResourceNotFound:
        return sdf_xml

    package_uri = "file://" + package_path.rstrip("/") + "/"
    return sdf_xml.replace("model://r1/", package_uri)


def convert_urdf_to_sdf(model_xml, model_name, canonical_link):
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as temp_file:
        temp_file.write(model_xml)
        temp_path = temp_file.name

    try:
        sdf_xml = subprocess.check_output(
            ["gz", "sdf", "-p", temp_path],
            stderr=subprocess.STDOUT,
            universal_newlines=True,
        )
    finally:
        try:
            os.unlink(temp_path)
        except OSError:
            pass

    root = ET.fromstring(sdf_xml)
    model = root.find("model")
    if model is None:
        raise ValueError("converted SDF does not contain a model element")

    model.set("name", model_name)
    model.set("canonical_link", canonical_link)
    return replace_model_uris(ET.tostring(root, encoding="unicode"))


def main():
    rospy.init_node("spawn_robot_model")

    model_name = rospy.get_param("~model_name", "R1")
    description_param = rospy.get_param("~description_param", "robot_description")
    robot_namespace = rospy.get_param("~robot_namespace", "/")
    reference_frame = rospy.get_param("~reference_frame", "world")
    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 0.001)
    yaw = rospy.get_param("~yaw", 0.0)
    retry_count = rospy.get_param("~retry_count", 3)
    retry_delay = rospy.get_param("~retry_delay", 0.5)
    spawn_format = rospy.get_param("~spawn_format", "sdf").strip().lower()
    canonical_link = rospy.get_param("~canonical_link", "base_footprint")
    strip_depth_camera = rospy.get_param("~strip_depth_camera", False)
    disable_laser_visualization = rospy.get_param("~disable_laser_visualization", False)

    if not rospy.has_param(description_param):
        rospy.logerr("spawn_robot_model could not find param %s", description_param)
        return 1

    model_xml = rospy.get_param(description_param)
    if not model_xml:
        rospy.logerr("spawn_robot_model got empty XML from %s", description_param)
        return 1

    try:
        model_xml = prepare_model_xml(
            model_xml,
            strip_depth_camera=strip_depth_camera,
            disable_laser_visualization=disable_laser_visualization,
        )
    except ET.ParseError as exc:
        rospy.logerr("spawn_robot_model could not parse %s: %s", description_param, exc)
        return 1

    if spawn_format == "sdf":
        try:
            model_xml = convert_urdf_to_sdf(model_xml, model_name, canonical_link)
            rospy.loginfo(
                "spawn_robot_model converted %s to SDF with canonical_link=%s",
                description_param,
                canonical_link,
            )
        except (subprocess.CalledProcessError, OSError, ValueError, ET.ParseError) as exc:
            rospy.logerr("spawn_robot_model could not convert %s to SDF: %s", description_param, exc)
            return 1
    elif spawn_format != "urdf":
        rospy.logerr("spawn_robot_model got unsupported spawn_format: %s", spawn_format)
        return 1

    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/get_world_properties")
    spawn_service_name = "/gazebo/spawn_sdf_model" if spawn_format == "sdf" else "/gazebo/spawn_urdf_model"
    rospy.wait_for_service(spawn_service_name)

    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    spawn_model = rospy.ServiceProxy(spawn_service_name, SpawnModel)

    try:
        world = get_world_properties()
        if model_name in world.model_names:
            delete_model(model_name)
            rospy.loginfo("spawn_robot_model deleted stale model %s before spawning", model_name)
            rospy.sleep(0.2)
    except rospy.ServiceException as exc:
        rospy.logdebug("spawn_robot_model pre-spawn cleanup skipped: %s", exc)

    pose = build_pose(x, y, z, yaw)

    for attempt in range(1, retry_count + 1):
        try:
            response = spawn_model(model_name, model_xml, robot_namespace, pose, reference_frame)
        except rospy.ServiceException as exc:
            rospy.logwarn(
                "spawn_robot_model attempt %d/%d failed with service error: %s",
                attempt,
                retry_count,
                exc,
            )
            rospy.sleep(retry_delay)
            continue

        if response.success:
            rospy.loginfo(
                "spawn_robot_model spawned %s at (%.3f, %.3f, %.3f, yaw=%.3f)",
                model_name,
                x,
                y,
                z,
                yaw,
            )
            return 0

        rospy.logwarn(
            "spawn_robot_model attempt %d/%d rejected: %s",
            attempt,
            retry_count,
            response.status_message,
        )

        if "already exists" in response.status_message.lower():
            try:
                delete_model(model_name)
                rospy.sleep(0.2)
            except rospy.ServiceException:
                pass

        rospy.sleep(retry_delay)

    rospy.logerr("spawn_robot_model failed to spawn %s after %d attempts", model_name, retry_count)
    return 1


if __name__ == "__main__":
    sys.exit(main())
