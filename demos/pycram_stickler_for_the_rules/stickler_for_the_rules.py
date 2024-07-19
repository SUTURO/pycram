from enum import Enum

from demos.pycram_hsrb_real_test_demos.restaurant import transform_pose
from pycram.external_interfaces import robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_serve_breakfast_demo.utils.misc import *
from pycram.utilities.robocup_utils import *
import pycram.external_interfaces.giskard_new as giskardpy

# from pycram.external_interfaces.knowrob import get_table_pose

human_pose = None

rooms_list = ["office", "kitchen", "living room", "hallway"]

# An instance of the TextToSpeechPublisher
text_to_speech_publisher = TextToSpeechPublisher()

# An instance of the ImageSwitchPublisher
image_switch_publisher = ImageSwitchPublisher()

# An instance of the StartSignalWaiter
start_signal_waiter = StartSignalWaiter()

# An instance of the PoseNavigator
move = PoseNavigator()

# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
# TODO: change urdf
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")

giskardpy.init_giskard_interface()
giskardpy.clear()
giskardpy.sync_worlds()


def try_detect():
    """
    lets the robot looks on a pose and perceive objects or free spaces

    :param pose: the pose that the robot looks to
    :param location: if location should be detected or not
    :return: tupel of State and dictionary of found objects in the FOV
    """
    image_switch_publisher.pub_now(10)
    text_to_speech_publisher.pub_now("Perceiving")
    try:
        object_desig = DetectAction(technique='holding_drink').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    image_switch_publisher.pub_now(0)
    return object_desig


def filter_humans(obj_dict: dict):
    """
    keeps only wished objects of the seen objects and sorts the returned list of objects
    according to the order of the given wished_sorted_obj_list.

    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :param wished_sorted_obj_list: list of object types we like to keep with the wished order
    :return: sorted list of seen and wished to keep objects in the same order of the given list
    """
    sorted_objects = []

    if len(obj_dict) == 0:
        return sorted_objects

    # cut of the given State and keep the dictionary
    first, *remaining = obj_dict
    for dictionary in remaining:
        for value in dictionary.values():
            sorted_objects.append(value)

    # print which objects are in the final list
    test_list = []
    for test_object in sorted_objects:
        test_list.append(test_object.type)
    print(test_list)

    return sorted_objects


def monitor_func():
    global human_pose
    robot_pose = robot.get_pose()
    distance = math.sqrt(pow((human_pose.pose.position.x - robot_pose.pose.position.x), 2) +
                         pow((human_pose.pose.position.y - robot_pose.pose.position.y), 2) +
                         pow((human_pose.pose.position.z - robot_pose.pose.position.z), 2))
    if distance < 1:
        return SensorMonitoringCondition
    return False


def drink_rule(room):
    global human_pose
    if room == "hallway":
        target_orientation = axis_angle_to_quaternion([0, 0, 1], 45)
        move.pub_now(Pose([2.53, 0.55, 0], [target_orientation[0], target_orientation[1],
                                            target_orientation[2], target_orientation[3]]))
    elif room == "office":
        room_rule()
        navigate_to(4.66, 3.65, "left")
        config = {'head_pan_joint': 0.2}
        pakerino(config=config)
        room_rule()
        config = {'head_pan_joint': 0.4}
        pakerino(config=config)
        room_rule()

    elif room == "kitchen":
        target_orientation = axis_angle_to_quaternion([0, 0, 1], 45)
        move.pub_now(Pose([2.53, 0.55, 0], [target_orientation[0], target_orientation[1],
                                            target_orientation[2], target_orientation[3]]))

    elif room == "living room":
        target_orientation = axis_angle_to_quaternion([0, 0, 1], 45)
        move.pub_now(Pose([2.53, 0.55, 0], [target_orientation[0], target_orientation[1],
                                            target_orientation[2], target_orientation[3]]))

    # TODO: wahrscheinlich lokkataction hinzufügen
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    humans_desig = try_detect()
    humans_list = filter_humans(humans_desig)

    for human in humans_list:
        holding_drink = eval(human.attribute[0])
        if not holding_drink:
            move_to_human(human)

        # TODO: Orientation anpassen
        LookAtAction([Pose([human_pose.pose.position.x, human_pose.pose.position.y, 0.12])]).resolve().perform()
        # LookAtAction([Pose([human_pose.pose.position.x, human_pose.pose.position.y, 0.12],
        # robot.get_pose().pose.orientation)]).resolve().perform()
        text_to_speech_publisher.pub_now("you should have a drink in your hand")
        text_to_speech_publisher.pub_now("please go to the kitchen")
        text_to_speech_publisher.pub_now("and take a drink")


def move_to_human(human):
    #### MOVING ####
    human_poseTm = transform_pose(human.pose, "head_rgbd_sensor_rgb_frame", "map")
    human_p = human_poseTm
    human_p.pose.position.z = 0
    human_p.pose.orientation.x = 0
    human_p.pose.orientation.y = 0
    human_p.pose.orientation.z = 0
    human_p.pose.orientation.w = 1

    human_pose = human_p

    try:
        plan = Code(lambda: move.pub_now(human_p)) >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        print("ABBRUCH")
        move.interrupt()
        #### MOVING ####


def room_rule():
    humans_list = try_detect()
    for human in humans_list:
        if 1.74 <= human.pose.position.x <= 5.22 and 1.6 <= human.pose.position.y <= 6.6:
            move_to_human(human)
            text_to_speech_publisher.pub_now("you are not allowed to be in this room")
            text_to_speech_publisher.pub_now("can you please move to another room")


def navigate_to(x: float, y: float, table_name: str):
    """
    Navigates to popcorn table, long table or shelf.

    :param x: x pose to navigate to
    :param y: y pose to navigate to
    :param table_name: defines the name of the table to move to
    """
    if table_name == "front":
        move.pub_now(Pose([x, y, 0], [0, 0, 0, 1]))
    elif table_name == "left":
        move.pub_now(Pose([x, y, 0], [0, 0, 0.7, 0.7]))
    elif table_name == "right":
        move.pub_now(Pose([x, y, 0], [0, 0, -0.7, -0.7]))
    elif table_name == "behind":
        move.pub_now(Pose([x, y, 0], [0, 0, 1, 0]))


with real_robot:
    rospy.loginfo("Starting demo")
    text_to_speech_publisher.pub_now("Starting demo")
    image_switch_publisher.pub_now(0)
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    navigate_to(4.86, 0.23, "left")

    for room in rooms_list:
        drink_rule(room)
