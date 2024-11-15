"""Implementation of helper functions and classes for internal usage only.

Classes:
Singleton -- implementation of singleton metaclass
"""

from typing import Tuple, List

import numpy as np


from pycram.datastructures.enums import Grasp
from pycram.local_transformer import LocalTransformer
from pycram.robot_description import RobotDescription

from scipy.spatial.transform import Rotation as R


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
        str: The corresponding face of the object.
    """
    max_index = np.argmax(np.abs(vector))
    max_sign = int(np.sign(vector[max_index]))
    axis = INDEX_TO_AXIS[max_index]

    return AXIS_INDEX_TO_FACE[(axis, max_sign)]


def calculate_object_faces(object):
    """
    Calculates the faces of an object relative to the robot based on orientation.

    This method determines the face of the object that is directed towards the robot,
    as well as the bottom face, by calculating vectors aligned with the robot's negative x-axis
    and negative z-axis in the object's frame.

    Args:
        object (Object): The object whose faces are to be calculated, with an accessible pose attribute.

    Returns:
        list: A list containing two Grasp Enums, where the first element is the face of the object facing the robot,
              and the second element is the top or bottom face of the object.
    """
    local_transformer = LocalTransformer()
    oTm = object.pose

    base_link = RobotDescription.current_robot_description.base_link

    oTb = local_transformer.transform_pose(oTm, object.world_object.world.robot.get_link_tf_frame(base_link))
    orientation = oTb.orientation_as_list()

    rotation_matrix = R.from_quat([orientation[0], orientation[1], orientation[2], orientation[3]]).inv().as_matrix()

    robot_negative_x_vector = -rotation_matrix[:, 0]
    robot_negative_z_vector = -rotation_matrix[:, 2]

    facing_robot_face = calculate_vector_face(robot_negative_x_vector)
    bottom_face = calculate_vector_face(robot_negative_z_vector)

    return [facing_robot_face, bottom_face]


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