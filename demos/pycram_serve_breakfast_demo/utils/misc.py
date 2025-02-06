import rospy
from giskardpy.data_types.exceptions import ForceTorqueThresholdException
from geometry_msgs.msg import PoseStamped
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.failures import EnvironmentUnreachable, GripperClosedCompletely
from pycram.worlds.bullet_world import BulletWorld


def sort_objects(obj_dict: dict, wished_sorted_obj_list: list):
    """
        keeps only wished objects of the seen objects and sorts the returned list of objects
        according to the order of the given wished_sorted_obj_list.
        :param obj_dict: tupel of State and dictionary of founded objects in the FOV
        :param wished_sorted_obj_list: list of object types we like to keep with the wished order
        :return: sorted list of seen and wished to keep objects in the same order of the given list
        """
    tuples_list = []
    sorted_objects = []
    if len(obj_dict) == 0:
        return sorted_objects

    for value in obj_dict.values():
        object_type = value.obj_type
        if value.obj_type in ["Mueslibox", "Cornybox", "Cerealbox", "Crackerbox", "MuesliboxVitalis"]:
            object_type = "Cerealbox"
        if value.obj_type in ["Spoon", "Fork", "Knife", "Plasticknife"]:
            object_type = "Spoon"
        if value.obj_type in ["Milkpack", "Milkpackja", "MilkpackLactoseFree"]:
            object_type = "Milkpack"
        if object_type in wished_sorted_obj_list:
            tuples_list.append((value, wished_sorted_obj_list.index(object_type)))
    sorted_objects = [x[0] for x in sorted(tuples_list, key=lambda index: index[1])]

    # print which objects are in the final list
    test_list = []
    for test_object in sorted_objects:
        test_list.append(test_object.obj_type)
    print(test_list)

    return sorted_objects


def get_bowl_list(obj_dict: dict):
    """
    searches in a dictionary of objects for all bowls and returns them
    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :return: list of found bowls
    """
    objects_list = []
    if len(obj_dict) == 0:
        return objects_list
    for value in obj_dict.values():
        if value.obj_type == "Metalbowl":
            objects_list.append(value)
    return objects_list


def get_bowl(obj_dict: dict):
    """
    searches in a dictionary of objects for a bowl and returns it
    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :return: the found bowl or None
    """
    if len(obj_dict) == 0:
        return None
    for value in obj_dict.values():
        if value.obj_type == "Metalbowl":
            return value
    return None


def get_free_spaces(location_list: list):
    """
    looks in a list of regions for regions that are free and returns them
    :param location_list: a list of regions
    :return: sorted list of found free regions
    """
    free_places_tuples = []
    sorted_places = []
    if len(location_list) == 0:
        return sorted_places
    for location in location_list:
        print(f"location: {location}, type: {type(location)}")
        seperated_location = location.split(',')
        occupied = eval(seperated_location[1])
        if not occupied:
            location_pose = PoseStamped()
            location_pose.header.frame_id = "/map"
            location_pose.pose.position.x = float(seperated_location[2])
            location_pose.pose.position.y = float(seperated_location[3])
            location_pose.pose.position.z = float(seperated_location[4])
            free_places_tuples.append((location_pose, location_pose.pose.position.y))
    sorted_list = sorted(free_places_tuples, key=lambda y_pose: y_pose[1])
    sorted_places = [tup[0] for tup in sorted_list]
    # print which objects are in the final list
    test_list = []
    for test_object in sorted_places:
        test_list.append(test_object.pose)
    print(test_list)
    return sorted_places


def try_pick_up(robot: BulletWorld.robot, obj: ObjectDesignatorDescription.Object, grasps: Grasp):
    """
    Picking up any object with failure handling.
    :param robot: the robot
    :param obj: the object that should be picked up
    :param grasps: how to pick up the object
    """
    try:
        PickUpAction(obj, [Arms.LEFT], [grasps]).resolve().perform()
    except (EnvironmentUnreachable, GripperClosedCompletely, ManipulationFTSCheckNoObject):
        TalkingMotion("Try pick up again").perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        # after failed attempt to pick up the object, the robot moves 30cm back on x pose
        step_back(robot)
        NavigateAction([Pose([obj.pose.position.x, 4, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
        # try to detect the object again
        LookAtAction(targets=[Pose([obj.pose.position.x, obj.pose.position.y, 0.21], [0, 0, 0.7, 0.7])]).resolve().perform()
        object_desig = DetectAction(technique='all').resolve().perform()
        new_object = sort_objects(object_desig, [obj.obj_type])[0]
        # second try to pick up the object
        try:
            TalkingMotion("try again").perform()
            PickUpAction(new_object, [Arms.LEFT], [grasps]).resolve().perform()
        # ask for human interaction if it fails a second time
        except (EnvironmentUnreachable, GripperClosedCompletely, ManipulationFTSCheckNoObject):
            step_back(robot)
            TalkingMotion(f"Can you please give me the {obj.obj_type} in the shelf?").perform()
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            rospy.sleep(4)
            MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()


def step_back(robot: BulletWorld.robot):
    """"
    steps back, parks arms and opens gripper
    """
    NavigateAction(
        [Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.3, 0],
              robot.get_pose().pose.orientation)]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()