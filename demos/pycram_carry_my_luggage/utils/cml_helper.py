from pycram.utilities.tf_wrapper import tf_listener


def transform_camera_to_x(robot, pose, frame_x):
    """
    transforms the pose with given frame_x, orientation will be head ori and z is minu 1.3
    """
    pose.pose.position.z -= 1.3
    pose.header.frame_id = "hsrb/" + frame_x
    tPm = tf_listener.transformPose("/map", pose)
    tPm.pose.position.z = 0
    pan_pose = robot.get_link_pose("head_pan_link")
    tPm.pose.orientation = pan_pose.pose.orientation
    return tPm