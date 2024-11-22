from typing import List, Optional, Union

import numpy as np
from scipy.spatial.transform import Rotation as R

from pycram.datastructures.enums import Grasp
from pycram.designator import ObjectDesignatorDescription
from pycram.robot_description import RobotDescription
from pycram.world_concepts.world_object import Object

#########################################
# Author: Luca Krohm
# Disclaimer:
# The code in this file was taken from a WIP version of Luca Krohm's Masters thesis
# so unexpected behaviour may occur. The code was used to fix the grasp calculations of the robot


def adjust_grasp_for_object_rotation(object, grasp, arm):
    """
    Adjusts the grasp orientation based on the object's current orientation.

    This function combines the specified grasp orientation with the object's current orientation
    to produce the final orientation needed for the end effector to grasp the object correctly.

    Args:
        object (Object): The object to be grasped, with an orientation accessible as a quaternion list.
        grasp (Enum): The specified grasp type, used to retrieve the predefined grasp orientation.
        arm (Enum): The arm used for grasping, needed to access the end effector's grasp orientations.

    Returns:
        list: A quaternion [x, y, z, w] representing the adjusted grasp orientation.

    # TODO: currently redundant, can also call pycram.datastructures.pose.Pose.multiply_quaternions with some preperation
    """
    grasp_orientation = RobotDescription.current_robot_description.grasps[grasp]
    x1, y1, z1, w1 = grasp_orientation
    x2, y2, z2, w2 = object.orientation_as_list()

    w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
    x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
    y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
    z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1

    return [x, y, z, w]


INDEX_TO_AXIS = {0: 'x', 1: 'y', 2: 'z'}
AXIS_TO_INDEX = {'x': 0, 'y': 1, 'z': 2}
AXIS_INDEX_TO_FACE = {
    ('x', -1): Grasp.FRONT,
    ('x', 1): Grasp.BACK,
    ('y', 1): Grasp.LEFT,
    ('y', -1): Grasp.RIGHT,
    ('z', -1): Grasp.TOP,
    ('z', 1): Grasp.BOTTOM
}

FACE_TO_AXIS_INDEX = {
    Grasp.FRONT: ('x', -1),
    Grasp.BACK: ('x', 1),
    Grasp.LEFT: ('y', 1),
    Grasp.RIGHT: ('y', -1),
    Grasp.TOP: ('z', -1),
    Grasp.BOTTOM: ('z', 1)
}


def calculate_vector_face(vector: List):
    """
    Determines the face of the object based on the input vector.

    Args:
        vector (List): A 3D vector representing one of the robot's axes in the object's frame.

    Returns:
        Grasp: The corresponding face of the object.
    """
    max_index = np.argmax(np.abs(vector))
    max_sign = int(np.sign(vector[max_index]))
    axis = INDEX_TO_AXIS[max_index]

    return AXIS_INDEX_TO_FACE[(axis, max_sign)]


def calculate_object_faces(target_object: ObjectDesignatorDescription.Object, robot: Optional[Object] = None) -> Grasp:
    """
    Calculates the faces of an object relative to the robot based on orientation and position.

    This method determines the faces of the object that are directed towards the robot, by calculating vectors from
    the object to the robot's base and using the object's orientation to determine the side and top/bottom faces.

    For side_face, only the x and y components are considered.
    For top_bottom_face, only the z component is considered.

    Args:
        object (ObjectDesignatorDescription.Object): The object whose faces are to be calculated, with an accessible pose attribute.

    Returns:
        list: A list containing two Grasp Enums, where the first element is the face of the object facing the robot,
              and the second element is the top or bottom face of the object.
    """

    obj_desig = target_object if isinstance(target_object,
                                            ObjectDesignatorDescription.Object) else target_object.resolve()

    oTm = obj_desig.pose
    base_link = RobotDescription.current_robot_description.base_link
    if robot is None:
        base_link_pose = obj_desig.world_object.world.robot.get_link_pose(base_link)
    else:
        base_link_pose = robot.get_link_pose(base_link)

    object_position = oTm.position_as_list()
    robot_position = base_link_pose.position_as_list()
    vector_to_robot_world = [robot_position[i] - object_position[i] for i in range(3)]

    orientation = oTm.orientation_as_list()
    rotation_matrix = R.from_quat(orientation).as_matrix()
    o_R_w = rotation_matrix.T

    vector_to_robot_local = o_R_w.dot(vector_to_robot_world)

    vector_x, vector_y, _ = vector_to_robot_local

    vector_facing = [vector_x, vector_y, 0]
    facing_robot_face = calculate_vector_face(vector_facing)

    return facing_robot_face
