import numpy as np
from hsrb_simple_actions import tf
from tf.transformations import quaternion_matrix

from pycram.datastructures.pose import Pose


def change_orientation(startPose: Pose):
    """
    Rotates the base of the HSR in a 180-degree rotation around the origin.
    :param startPose: Pose to rotate around.
    :return: Rotated Pose.
    """
    quat_orientation = (startPose.pose.orientation.x, startPose.pose.orientation.y, startPose.pose.orientation.z,
              startPose.pose.orientation.w)
    quat_add = tf.transformations.quaternion_from_euler(0, 0, np.pi)
    quat_add_new = (quat_add[0], quat_add[1], quat_add[2], quat_add[3])

    q_new = tf.transformations.quaternion_multiply(quat_orientation, quat_add_new)
    new_angle = (q_new[0], q_new[1], q_new[2], q_new[3])
    newPose = Pose([startPose.pose.position.x, startPose.pose.position.y, startPose.pose.position.z],
                   [new_angle[0], new_angle[1], new_angle[2], new_angle[3]])
    return newPose


def move_pose_forwards(goal: Pose, distance: float):
    """
    Moves the goal pose of a navigation action on a vector given the set distance.
    :param goal: Pose to move.
    :param distance: Distance to move the goal.
    :return: Moved Pose.
    """
    rotMatrix = quaternion_matrix([goal.pose.orientation.x,
                                   goal.pose.orientation.y,
                                   goal.pose.orientation.z,
                                   goal.pose.orientation.w])
    forward_vector = rotMatrix[:3, 0]
    movedPose = np.array([goal.pose.position.x,
                      goal.pose.position.y,
                      goal.pose.position.z]) - distance * forward_vector
    return movedPose

def set_pose_in_front(goalPose: Pose, dist : float):
    new_pos = move_pose_forwards(goalPose, dist)
    adjusted_pose = Pose(position=[new_pos[0], new_pos[1], new_pos[2]], orientation=[goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])
    return adjusted_pose



