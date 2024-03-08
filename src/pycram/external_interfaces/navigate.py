import rospy
import actionlib

from ..designator import ObjectDesignatorDescription
from ..pose import Pose
from ..local_transformer import LocalTransformer
from ..bullet_world import BulletWorld
from ..enums import ObjectType
from typing import Any
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal


def queryPoseNav(navpose):
    """
    Sends a goal to the move_base Service. Connection to Giskard interface,
    will move robot to the given pose: navpose
    :param navpose: PoseStamped pose robot will navigate to
    """

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Move base")

    def done_callback(state, result):
        rospy.loginfo("Finished moving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    goal_msg = MoveBaseGoal()

    goal_msg.target_pose = navpose
    rospy.loginfo("navigating")
    client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(goal_msg, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    return query_result
