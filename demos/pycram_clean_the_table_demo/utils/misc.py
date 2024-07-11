import rospy
from docutils.nodes import math
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.designators.object_designator import *
from std_msgs.msg import String
from pycram.plan_failures import EnvironmentUnreachable, GripperClosedCompletely
from pycram.language import Monitor, Code
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

text_to_speech_publisher = TextToSpeechPublisher()
# ForceTorqueSensor for recognizing push on the hand
fts = ForceTorqueSensor(robot_name='hsrb')


def sort_objects(robot: BulletWorldObject, obj_dict: dict, wished_sorted_obj_list: list):
    """
    Transforms the given object dictionary to a distance sorted list.
    The Metalplate, if seen, is arranged as the last object in the list.

    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :param wished_sorted_obj_list: list of object types we like to keep
    :param robot: the robot
    :return: distance sorted list of seen and wished to keep objects
    """
    sorted_objects = []
    object_dict = {}
    containsPlate = False
    metalplate = None

    if len(obj_dict) == 0:
        return sorted_objects

    # cut of the given State and keep the dictionary
    first, *remaining = obj_dict
    robot_pose = robot.get_pose()
    # calculate euclidian distance for all found object in a dictionary
    for dictionary in remaining:
        for value in dictionary.values():
            distance = math.sqrt(pow((value.pose.position.x - robot_pose.pose.position.x), 2) +
                                 pow((value.pose.position.y - robot_pose.pose.position.y), 2) +
                                 pow((value.pose.position.z - robot_pose.pose.position.z), 2))

            # fill dict with objects and their distance.
            # Add seen objects only once, if seen multiple times
            if value.type not in object_dict:
                object_dict[value.type] = (value, distance)

    # sort all objects that where in the obj_dict
    sorted_object_list = sorted(object_dict.items(), key=lambda distance: distance[1][1])
    sorted_object_dict = dict(sorted_object_list)

    # keep only objects that are in the wished list and put Metalplate last
    for (object, distance) in sorted_object_dict.values():
        if object.type in wished_sorted_obj_list:
            if object.type == "Metalplate":
                containsPlate = True
                metalplate = object
            else:
                sorted_objects.append(object)

    # when all objects are sorted in the list add the plate at last
    if containsPlate:
        # or making Metalplate to the first object in list: sorted_objects.insert(0, metalplate)
        sorted_objects.append(metalplate)

    # print which objects are in the final list
    test_list = []
    for test_object in sorted_objects:
        test_list.append(test_object.type)
    print(test_list)

    return sorted_objects


def try_pick_up(robot: BulletWorld.robot, obj: ObjectDesignatorDescription, grasps: str):
    """
    Picking up any object with failure handling.

    :param robot: the robot
    :param obj: the object that should be picked up
    :param grasps: how to pick up the object
    """
    try:
        PickUpAction(obj, ["left"], [grasps]).resolve().perform()
    except (EnvironmentUnreachable, GripperClosedCompletely):
        # print("try pick up again")
        # TalkingMotion("Try pick up again").resolve().perform()
        # # after failed attempt to pick up the object, the robot moves 30cm back on x pose
        # # TODO: x-pose and orientation should be generalized
        # NavigateAction(
        #     [Pose([robot.get_pose().position.x + 0.3, robot.get_pose().position.y,
        #            robot.get_pose().position.z], [0, 0, 1, 0])]).resolve().perform()
        # ParkArmsAction([Arms.LEFT]).resolve().perform()
        # # try to detect the object again
        # object_desig = DetectAction(technique='default').resolve().perform()
        # new_object = sort_objects(robot, object_desig, [obj.type])[0]
        #
        # # second try to pick up the object
        # try:
        #     TalkingMotion("try again").resolve().perform()
        #     PickUpAction(new_object, ["left"], [grasps]).resolve().perform()
        # ask for human interaction if it fails a second time
        # except (EnvironmentUnreachable, GripperClosedCompletely):
        # NavigateAction(
        #     [Pose([robot.get_pose().position.x + 0.3, robot.get_pose().position.y,
        #            robot.get_pose().position.z], [0, 0, 1, 0])]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        text_to_speech_publisher.pub_now(f"Can you pleas give me the {obj.type} on the table?")
        MoveGripperMotion("open", "left").resolve().perform()
        text_to_speech_publisher.pub_now("Push down my hand, when I should grasp")
        try:
            plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            rospy.logwarn("Close Gripper")
            text_to_speech_publisher.pub_now("Grasping.")  # Todo could be deleted if demo to long
            MoveGripperMotion(motion="close", gripper="left").resolve().perform()


def monitor_func():
    der: WrenchStamped() = fts.get_last_value()
    print(abs(der.wrench.force.x))
    if abs(der.wrench.force.x) > 10.30:
        print(abs(der.wrench.force.x))
        print(abs(der.wrench.torque.x))
        return SensorMonitoringCondition
    return False
