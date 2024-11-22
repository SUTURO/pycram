"""Implementation of helper functions and classes for internal usage only.

Classes:
Singleton -- implementation of singleton metaclass
"""
import math
import os
from typing import Tuple, List

import numpy as np
from geometry_msgs.msg import Quaternion
from typing_extensions import Dict, Optional
import xml.etree.ElementTree as ET

from pycram.datastructures.pose import Pose
from pycram.ros.logging import logwarn


class Singleton(type):
    """
    Metaclass for singletons
    """

    _instances = {}
    """
    Dictionary of singleton child classes inheriting from this metaclass, keyed by child class objects.
    """

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


def parse_mjcf_actuators(file_path: str) -> Dict[str, str]:
    """
    Parse the actuator elements from an MJCF file.

    :param file_path: The path to the MJCF file.
    """
    tree = ET.parse(file_path)
    root = tree.getroot()

    joint_actuators = {}

    # Iterate through all actuator elements
    for actuator in root.findall(".//actuator/*"):
        name = actuator.get('name')
        joint = actuator.get('joint')
        if name and joint:
            joint_actuators[joint] = name

    return joint_actuators


def get_robot_mjcf_path(robot_relative_dir: str, robot_name: str, xml_name: Optional[str] = None) -> Optional[str]:
    """
    Get the path to the MJCF file of a robot.

    :param robot_relative_dir: The relative directory of the robot in the Multiverse resources/robots directory.
    :param robot_name: The name of the robot.
    :param xml_name: The name of the XML file of the robot.
    :return: The path to the MJCF file of the robot if it exists, otherwise None.
    """
    xml_name = xml_name if xml_name is not None else robot_name
    if '.xml' not in xml_name:
        xml_name = xml_name + '.xml'
    multiverse_resources = find_multiverse_resources_path()
    try:
        robot_folder = os.path.join(multiverse_resources, 'robots', robot_relative_dir, robot_name)
    except TypeError:
        logwarn("Multiverse resources path not found.")
        return None
    if multiverse_resources is not None:
        list_dir = os.listdir(robot_folder)
        if 'mjcf' in list_dir:
            if xml_name in os.listdir(robot_folder + '/mjcf'):
                return os.path.join(robot_folder, 'mjcf', xml_name)
        elif xml_name in os.listdir(robot_folder):
            return os.path.join(robot_folder, xml_name)
    return None


def find_multiverse_resources_path() -> Optional[str]:
    """
    :return: The path to the Multiverse resources directory.
    """
    # Get the path to the Multiverse installation
    multiverse_path = find_multiverse_path()

    # Check if the path to the Multiverse installation was found
    if multiverse_path:
        # Construct the path to the resources directory
        resources_path = os.path.join(multiverse_path, 'resources')

        # Check if the resources directory exists
        if os.path.exists(resources_path):
            return resources_path

    return None


def find_multiverse_path() -> Optional[str]:
    """
    :return: the path to the Multiverse installation.
    """
    # Get the value of PYTHONPATH environment variable
    pythonpath = os.getenv('PYTHONPATH')
    multiverse_relative_path = "Multiverse/multiverse"

    # Check if PYTHONPATH is set
    if pythonpath:
        # Split the PYTHONPATH into individual paths using the platform-specific path separator
        paths = pythonpath.split(os.pathsep)

        # Iterate through each path and check if 'Multiverse' is in it
        for path in paths:
            if multiverse_relative_path in path:
                multiverse_path = path.split(multiverse_relative_path)[0]
                return multiverse_path + multiverse_relative_path


def axis_angle_to_quaternion(axis: List, angle: float) -> Tuple:
    """
    Convert axis-angle to quaternion.
    :param axis: (x, y, z) tuple representing rotation axis.
    :param angle: rotation angle in degree
    :return: The quaternion representing the axis angle
    """
    angle = math.radians(angle)
    axis_length = math.sqrt(sum([i ** 2 for i in axis]))
    normalized_axis = tuple(i / axis_length for i in axis)
    x = normalized_axis[0] * math.sin(angle / 2)
    y = normalized_axis[1] * math.sin(angle / 2)
    z = normalized_axis[2] * math.sin(angle / 2)
    w = math.cos(angle / 2)
    return (x, y, z, w)


def multiply_quaternions(q1: List, q2: List) -> List:
    """
    Multiply two quaternions using the robotics convention (x, y, z, w).
    :param q1: The first quaternion
    :param q2: The second quaternion
    :return: The quaternion resulting from the multiplication
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


def quaternion_rotate(q: List, v: List) -> List:
    """
    Rotate a vector v using quaternion q.
    :param q: A quaternion of how v should be rotated
    :param v: A vector that should be rotated by q
    :return: V rotated by Q as a quaternion
    """
    q_conj = (-q[0], -q[1], -q[2], q[3])  # Conjugate of the quaternion
    v_quat = (*v, 0)  # Represent the vector as a quaternion with w=0
    return multiply_quaternions(multiply_quaternions(q, v_quat), q_conj)[:3]


def multiply_poses(pose1: Pose, pose2: Pose) -> Tuple:
    """
    Multiply two poses.
    :param pose1: first Pose that should be multiplied
    :param pose2: Second Pose that should be multiplied
    :return: A Tuple of position and quaternion as result of the multiplication
    """
    pos1, quat1 = pose1.pose.position, pose1.pose.orientation
    pos2, quat2 = pose2.pose.position, pose2.pose.orientation
    # Multiply the orientations
    new_quat = multiply_quaternions(quat1, quat2)
    # Transform the position
    new_pos = np.add(pos1, quaternion_rotate(quat1, pos2))

    return new_pos, new_quat


def urdf_to_string(urdf_file_path):
    try:
        with open(urdf_file_path, 'r') as file:
            urdf_string = file.read()
        return urdf_string
    except FileNotFoundError:
        print(f"The file {urdf_file_path} was not found.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None


def norm_quaternion(q):
    """Return the norm (magnitude) of the quaternion q."""
    x, y, z, w = q
    return math.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2)


def normalize_quaternion(q):
    """Return the normalized quaternion q."""
    x, y, z, w = q
    norm = norm_quaternion(q)
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_to_angle(q):
    """Convert a quaternion to an angle of rotation in degrees."""
    # Normalize the quaternion
    q = normalize_quaternion(q)

    # Extract the scalar part (w) of the quaternion
    w = q[3]

    # Compute the angle in radians and then convert to degrees
    angle_radians = 2 * math.acos(w)
    angle_degrees = math.degrees(angle_radians)

    return angle_degrees
