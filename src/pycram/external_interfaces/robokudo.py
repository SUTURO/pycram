import sys
import time
from threading import Lock, RLock
from typing import Any, Union

import rospy

from ..datastructures.enums import RobokudoAnnotator, Demos
from ..ros.action_lib import create_action_client
from ..ros.logging import logwarn, loginfo, loginfo_once
from ..ros.ros_tools import get_node_names

from geometry_msgs.msg import PointStamped
from typing_extensions import List, Callable, Optional

from ..datastructures.pose import Pose
from ..designator import ObjectDesignatorDescription

try:
    from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjectDesignator
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
except ModuleNotFoundError as e:
    logwarn("Failed to import Robokudo messages, the real robot will not be available")

is_init = False

number_of_par_goals = 0
robokudo_lock = Lock()
robokudo_rlock = RLock()
with robokudo_rlock:
    par_threads = {}
    par_motion_goal = {}


def thread_safe(func: Callable) -> Callable:
    """
    Adds thread safety to a function via a decorator. This uses the robokudo_lock

    :param func: Function that should be thread safe
    :return: A function with thread safety
    """

    def wrapper(*args, **kwargs):
        with robokudo_rlock:
            return func(*args, **kwargs)

    return wrapper


def init_robokudo_interface(func: Callable) -> Callable:
    """
    Checks if the ROS messages are available and if Robokudo is running, if that is the case the interface will be
    initialized.

    :param func: Function this decorator should be wrapping
    :return: A callable function which initializes the interface and then calls the wrapped function
    """

    def wrapper(*args, **kwargs):
        global is_init
        if is_init and "/robokudo" in get_node_names():
            return func(*args, **kwargs)
        elif is_init and "/robokudo" not in get_node_names():
            logwarn("Robokudo node is not available anymore, could not initialize robokudo interface")
            is_init = False
            return

        if "robokudo_msgs" not in sys.modules:
            logwarn("Could not initialize the Robokudo interface since the robokudo_msgs are not imported")
            return

        if "/robokudo" in get_node_names():
            loginfo_once("Successfully initialized Robokudo interface")
            is_init = True
        else:
            logwarn("Robokudo is not running, could not initialize Robokudo interface")
            return
        return func(*args, **kwargs)

    return wrapper


@init_robokudo_interface
def send_query(obj_type: Optional[str] = None, region: Optional[str] = None,
               attributes: Optional[List[str]] = None) -> Any:
    """Generic function to send a query to RoboKudo."""
    goal = QueryGoal()

    if obj_type:
        goal.obj.type = obj_type
    if region:
        goal.obj.location = region
    if attributes:
        goal.obj.attribute = attributes

    client = create_action_client("robokudo/query", QueryAction)
    loginfo("Waiting for action server")
    client.wait_for_server()

    global human_bool
    global human_pose
    human_bool = False

    query_result = None

    def done_callback(state, result):
        nonlocal query_result
        query_result = result
        loginfo("Query completed")

    def human_callback(pose):
        global human_bool
        global human_pose
        human_bool = True
        human_pose = pose

    if goal.obj.type == "human":
        client.send_goal(goal)

        rospy.Subscriber("/human_pose", PointStamped, human_callback)

        while not human_bool:
            rospy.sleep(0.5)

        return human_pose

    client.send_goal(goal, done_cb=done_callback)
    client.wait_for_result()

    return query_result


@init_robokudo_interface
def query_object(obj_desc: ObjectDesignatorDescription) -> dict:
    """Query RoboKudo for an object that fits the description."""
    goal = QueryGoal()
    goal.obj.uid = str(id(obj_desc))

    result = send_query(obj_type=goal.obj.type)

    pose_candidates = {}
    if result and result.res:
        for i in range(len(result.res[0].pose)):
            pose = Pose.from_pose_stamped(result.res[0].pose[i])
            source = result.res[0].pose_source[0]
            pose_candidates[source] = pose
    return pose_candidates


@init_robokudo_interface
def query_human() -> PointStamped:
    """Query RoboKudo for human detection and return the detected human's pose."""
    result = send_query(obj_type='human')
    if result:
        return result  # Assuming result is of type PointStamped or similar.
    return "human found"


@init_robokudo_interface
def stop_query():
    """Stop any ongoing query to RoboKudo."""
    client = create_action_client('robokudo/query', QueryAction)
    client.wait_for_server()
    client.cancel_all_goals()
    loginfo("Cancelled current RoboKudo query goal")


@init_robokudo_interface
def query_faces_human() -> Any:
    """
    Sends a query to RoboKudo to look for a human. returns ID of the perceived human.
    """
    return send_query()


@init_robokudo_interface
def query_specific_region(region: str) -> Any:
    """Query RoboKudo to scan a specific region."""
    return send_query(region=region)


@init_robokudo_interface
def query_human_attributes() -> Any:
    """Query RoboKudo for human attributes like brightness of clothes, headgear, and gender."""
    result = send_query(attributes=["attributes"])
    if result:
        return result
    else:
        return None


@init_robokudo_interface
def query_waving_human() -> Pose:
    """Query RoboKudo for detecting a waving human."""
    result = send_query(obj_type='waving')
    if result:
        return result


def query_beverages() -> Any:
    """Query RoboKudo for detecting drinks in the robots field of vision"""
    return send_query(obj_type='beverage')


def get_annotator_topic(annotator_name: str, is_rwpipeline_path=True) -> str:
    """
    Returns the whole ROS topic for a given annotator output name of robokudo.

    Note: It does not check if the topic actually exists. The name is just appended.
    """
    if is_rwpipeline_path:
        return f"/robokudo/RWPipeline/{annotator_name}/output_image"

    return annotator_name

def get_annotators_of_demo(demo: Demos):
    demo_name = demo.name
    if demo == Demos.STORING_GROCERIES:
        annotator_result: List[RobokudoAnnotator] = [RobokudoAnnotator.YOLOANNOTATOR]
    elif demo == Demos.RECEPTIONIST:
        annotator_result: List[RobokudoAnnotator] = [RobokudoAnnotator.YOLOANNOTATOR]
    elif demo == Demos.CLEAN_THE_TABLE:
        annotator_result: List[RobokudoAnnotator] = [RobokudoAnnotator.YOLOANNOTATOR]
    elif demo == Demos.RESTAURANT:
        annotator_result: List[RobokudoAnnotator] = [RobokudoAnnotator.WAVING]
    else:
        logwarn(f"Demo {demo_name} does not have assigned annotators yet")
        return []

    return annotator_result

def get_used_annotator_list(annotators: Union[List[RobokudoAnnotator], Demos], as_topic_names=True) -> List[str]:
    if not annotators:
        logwarn("No annotators or demo preset provided for annotator names")
        return []

    annotator_result = []
    if isinstance(annotators, List):
        list_is_robokudo_annotator = [isinstance(an, RobokudoAnnotator) for an in annotators]

        if all(list_is_robokudo_annotator):
            annotator_result: List[RobokudoAnnotator] = annotators

    if isinstance(annotators, Demos):
        loginfo(f"Setting preset for: {annotators.name}")
        annotator_result = get_annotators_of_demo(annotators)

    annotator_strings: List[str] = [ann.value for ann in annotator_result]

    if as_topic_names:
        annotator_topics = [get_annotator_topic(annotator) for annotator in annotator_strings]
        annotator_strings = annotator_topics

    return annotator_strings