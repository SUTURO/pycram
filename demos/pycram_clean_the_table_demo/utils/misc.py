from typing_extensions import Optional

from demos.pycram_clean_the_table_demo.clean_the_table_intern_go import NavigatePose
from pycram.designators.action_designator import *
from pycram.failures import PerceptionObjectNotFound, EnvironmentUnreachable, GripperClosedCompletely
from pycram.worlds.bullet_world import BulletWorld

wished_sorted_obj_list_clean_the_table = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]

wished_sorted_obj_list_serve_breakfast = ["Metalbowl", "Cerealbox", "Milkpack", "Spoon"]


def get_objects(obj_dict: dict):
    objects_list = []

    if len(obj_dict) == 0:
        return objects_list

    for value in obj_dict.values():
        objects_list.append(value)

    return objects_list


def sort_objects_euclidian(robot: BulletWorld.robot, found_objects_list: list, wished_sorted_obj_list: list):
    """
    Transforms the given object dictionary to a distance sorted list.
    The Metalplate, if seen, is arranged as the last object in the list.
    :param found_objects_list: list of found objects in the FOV
    :param wished_sorted_obj_list: list of object types we like to keep
    :param robot: the robot
    :return: distance sorted list of seen and wished to keep objects
    """
    sorted_objects = []
    object_dict = {}
    containsPlate = False
    metalplate = None

    if len(found_objects_list) == 0:
        return sorted_objects

    # cut of the given State and keep the dictionary
    robot_pose = robot.get_pose()
    # calculate euclidian distance for all found object in a dictionary
    # for dictionary in remaining:
    for value in found_objects_list:
        distance = math.sqrt(pow((value.pose.position.x - robot_pose.pose.position.x), 2) +
                             pow((value.pose.position.y - robot_pose.pose.position.y), 2) +
                             pow((value.pose.position.z - robot_pose.pose.position.z), 2))

        # fill dict with objects and their distance.
        # Add seen objects only once, if seen multiple times
        if value.obj_type not in found_objects_list:
            object_dict[value.obj_type] = (value, distance)

    # sort all objects that where in the obj_dict
    sorted_object_list = sorted(object_dict.items(), key=lambda distance: distance[1][1])
    sorted_object_dict = dict(sorted_object_list)

    # keep only objects that are in the wished list and put Metalplate last
    for (object, distance) in sorted_object_dict.values():
        if object.obj_type in wished_sorted_obj_list:
            if object.obj_type == "Metalplate":
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
        test_list.append(test_object.obj_type)
    print(test_list)

    return sorted_objects


def try_detect(pose: Pose, technique: Optional[str] = None):
    """
    lets the robot looks on a pose and perceive objects or free spaces
    :param pose: the pose that the robot looks to
    :param technique: if location should be detected or not
    :return: tupel of State and dictionary of found objects in the FOV
    """
    LookAtAction(targets=[pose]).resolve().perform()
    TalkingMotion("Perceiving").perform()
    try:
        if technique == "location":
            object_desig = DetectAction(technique='location', state='popcorn_table').resolve().perform()
        else:
            object_desig = DetectAction(technique='all').resolve().perform()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def object_found(obj_dict: dict, obj_name: str) -> bool:
    """
    checks if an object is in the dictionary or not
    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :param obj_name: the object being checked
    :return: if the object has been found
    """
    if len(obj_dict) == 0:
        return False
    for value in obj_dict.values():
        if value.obj_type == obj_name:
            return True
    return False


def get_object(obj_dict: dict, obj_name: str):
    """
    checks if an object is in the dictionary or not
    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :param obj_name: the object being checked
    :return: if the object has been found
    """
    if len(obj_dict) == 0:
        return None
    for value in obj_dict.values():
        if value.obj_type == obj_name:
            return value
    return None


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


def try_pick_up_c(robot: BulletWorld.robot, obj: ObjectDesignatorDescription.Object, grasps: Grasp):
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
        NavigateAction([Pose([robot.get_pose().pose.position.x, NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                             NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
        # try to detect the object again
        object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 4.35, 0.35], [0, 0, -0.7, 0.7]))
        new_object = sort_objects_euclidian(robot, object_desig, [obj.obj_type])[0]
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
        [Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y + 0.3, 0],
              robot.get_pose().pose.orientation)]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
