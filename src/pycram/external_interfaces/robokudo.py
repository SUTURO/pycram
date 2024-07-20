import rospy
import actionlib
from tmc_msgs.msg import Voice

from ..designator import ObjectDesignatorDescription
from ..pose import Pose
from ..local_transformer import LocalTransformer
from ..bullet_world import BulletWorld
from ..enums import ObjectType
from typing import Any
from geometry_msgs.msg import PoseStamped, PointStamped

is_init = False


def init_robokudo_interface():
    global is_init
    if is_init:
        return
    try:
        from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjectDesignator
        from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
        is_init = True
        rospy.loginfo("Successfully initialized robokudo interface")
    except ModuleNotFoundError as e:
        rospy.logwarn(f"Could not import RoboKudo messages, RoboKudo interface could not be initialized")


def msg_from_obj_desig(obj_desc: ObjectDesignatorDescription) -> 'robokudo_ObjectDesignator':
    """
    Creates a RoboKudo Object designator from a PyCRAM Object Designator description

    :param obj_desc: The PyCRAM Object designator that should be converted
    :return: The RobotKudo Object Designator for the given PyCRAM designator
    """
    obj_msg = robokudo_ObjectDesignator()
    obj_msg.uid = str(id(obj_desc))
    obj_msg.type = obj_desc.types[0]  # For testing purposes

    return obj_msg


def make_query_goal_msg(obj_desc: ObjectDesignatorDescription) -> 'QueryGoal':
    """
    Creates a QueryGoal message from a PyCRAM Object designator description for the use of Querying RobotKudo.

    :param obj_desc: The PyCRAM object designator description that should be converted
    :return: The RoboKudo QueryGoal for the given object designator description
    """
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    goal_msg = QueryGoal()
    goal_msg.obj.uid = str(id(obj_desc))
    goal_msg.obj.type = str(obj_desc.types[0].name)  # For testing purposes
    if ObjectType.JEROEN_CUP == obj_desc.types[0]:
        goal_msg.obj.color.append("blue")
    elif ObjectType.BOWL == obj_desc.types[0]:
        goal_msg.obj.color.append("red")
    return goal_msg


def query(object_desc: ObjectDesignatorDescription) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    object_goal = make_query_goal_msg(object_desc)

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    wait = client.wait_for_result()
    pose_candidates = {}
    # todo check if query is even filled
    for i in range(0, len(query_result.res[0].pose)):
        pose = Pose.from_pose_stamped(query_result.res[0].pose[i])
        # todo check if frame exist and if not in map
        # pose.frame = BulletWorld.current_bullet_world.robot.get_link_tf_frame(pose.frame)
        source = query_result.res[0].pose_source[0]

        # lt = LocalTransformer()
        # pose = lt.transform_pose(pose, "map")

        pose_candidates[source] = pose

    return pose_candidates


def queryEmpty(object_desc: ObjectDesignatorDescription) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    object_goal = goal_msg = QueryGoal()
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    wait = client.wait_for_result()
    # pose_candidates = {}
    # #todo check if query is even filled
    
    return query_result


def queryRegion(region: str) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo to scan a specific region")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global query_result
        #Todo is result.res needed instead?
        query_result = result.res

    def feedback_callback(msg):
        pass

    object_goal = QueryGoal()
    object_goal.obj.location = str(region)
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    return query_result

def queryHuman() -> Any:
    """
    Sends a query to RoboKudo to look for a Human
    returns a PointStamped of pose where human is. keeps publishing it onto the
    topic /human_pose
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global human_bool
    global query_result
    global human_pose
    def active_callback():
        rospy.loginfo("Send query to Robokudo to perceive a human")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global human_bool
        human_bool = True
        global query_result
        query_result = result

    def feedback_callback(msg):
        #rospy.loginfo("Got feedback")
        global feedback_result
        feedback_result = msg

    def callback(pose):
        global human_bool
        global human_pose
        human_bool = True
        human_pose = pose

    # create client to communicate with perception
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    object_goal = goal_msg = QueryGoal()
    #object_goal.type = 'detect'
    object_goal.obj.type = 'human'
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)

    # if no human is detected
    human_bool = False
    waiting_human = False
    rospy.Subscriber("/human_pose", PointStamped, callback)

    while not human_bool:
        rospy.sleep(0.5)

    return human_pose


def stop_queryHuman() -> Any:
    """
       Sends a query to RoboKudo to stop human detection
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    client.wait_for_server()
    client.cancel_all_goals()
    print("get status: " + client.get_goal_status_text())
    rospy.loginfo("cancelled current goal")


def seat_queryHuman(seat: str) -> Any:
    """
    Sends a query to RoboKudo to check if a place is free to sit
    :param seat: name of the seat/region, that will be checked
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo to scan for seat and human")

    def done_callback(state, result: QueryResult):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result.res

    # fill Query with information so that perception looks for a seat
    object_goal = QueryGoal()

    object_goal.obj.location = str(seat) # aktivate region filter

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
    # TODO: necessary?
    client.wait_for_result()

    return query_result


def faces_queryHuman() -> Any:
    """
    Sends a query to RoboKudo to look for a human. returns four attributes of the perceived human.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo for face recognition")

    def done_callback(state, result: QueryResult):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    object_goal = QueryGoal()

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
    client.wait_for_result()

    return query_result


def attributes_queryHuman() -> Any:
    """
    Sends a query to RoboKudo to look for a human. returns four attributes of the perceived human.
    """
    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo to look for human and attributes")

    def done_callback(state, result: QueryResult):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    object_goal = QueryGoal()
    # Perception will detect brightness of clothes, kind of clothes, headgear and gender
    object_goal.obj.attribute = ["attributes"]

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
    # TODO: necessary?
    client.wait_for_result()

    return query_result

def query_waving_human() -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
    """
    #init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    object_goal = QueryGoal()

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    wait = client.wait_for_result()
    pose_candidate = None
    # todo check if query is even filled
    try:
        for i in range(0, len(query_result.res[0].pose)):
            pose = Pose.from_pose_stamped(query_result.res[0].pose[i])
            pose_candidate = pose
    except IndexError:
        pass
    return pose_candidate


def queryDrinkingHuman() -> ObjectDesignatorDescription:
    """
    Sends a query to RoboKudo to look for a human. returns list with all detected humans
    and fpr each human if they are holding a drink
    """
    print("robokudo query call")

    init_robokudo_interface()
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo for drinking human recognition")

    def done_callback(state, result: QueryResult):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    object_goal = QueryGoal()
    object_goal.obj.type = "drink"

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
    client.wait_for_result()

    return query_result
