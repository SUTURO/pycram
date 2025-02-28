from typing_extensions import Optional

from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from pycram.external_interfaces import giskard
from pycram.failures import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_clean_the_table_demo.utils.misc import *
from demos.pycram_serve_breakfast_demo.utils.misc import try_pick_up, sort_objects
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.utilities.robocup_utils import ImageSendPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

DRINKS = ["RedBullCan", "Milkpack", "MilkpackLactoseFree", "Milkpackja", "Cola"]

# Wished objects for the Demo
# wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]
wished_sorted_obj_list = ["RedBullCan"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the popcorn table
table_pose = 4.7

# name of the dishwasher handle and dishwasher door
handle_name = "iai_kitchen/sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

apart_desig = BelieveObject(names=["kitchen"])


# TODO: Enum for navigating
class NavigatePose(Enum):
    DISHWASHER_CLOSED = Pose([2.75, -2.1, 0], [0, 0, -1, 1])
    DISHWASHER = Pose([2.65, -1.85, 0], [0, 0, -1, 1])
    SHELF = Pose([4.5, 3.95, 0], [0, 0, 0, 1])
    POPCORN_TABLE = Pose([1.95, 4, 0], [0, 0, 0.7, 0.7])
    LONG_TABLE = Pose([1.7, 0.8, 0], [0, 0, 1, 0])


#TODO: x + 0.19, y -0.02

class PlacingXPose(Enum):
    """
    Differentiate the x pose for placing
    """
    CUTLERY = 2.64  # 2.376
    SPOON = 2.64  # 2.376
    FORK = 2.64  # 2.376
    PLASTICKNIFE = 2.64  # 2.376
    KNIFE = 2.64  # 2.376
    METALBOWL = 2.95  # 2.83
    METALMUG = 2.91  # 2.79
    METALPLATE = 2.92  # 2.8


class PlacingYPose(Enum):
    """
    Differentiate the y pose for placing
    """
    CUTLERY = -2.59  # -1.59 # mitte
    SPOON = -2.59  # -1.59
    FORK = -2.59  # -1.59
    PLASTICKNIFE = -2.59  # -1.59
    KNIFE = -2.59  # -1.59
    METALBOWL = -2.64 # -1.73
    METALMUG = -2.59  # -1.75
    METALPLATE = -2.72  # -1.65


class PlacingZPose(Enum):
    """
    Differentiate the z pose for placing
    """
    METALPLATE = 0.488
    OTHER = 0.5


def turn_around():
    if robot.get_pose().pose.orientation == NavigatePose.POPCORN_TABLE.value.pose.orientation:
        NavigateAction([Pose(robot.get_pose().pose.position,
                             NavigatePose.DISHWASHER.value.pose.orientation)]).resolve().perform()
    elif robot.get_pose().pose.orientation == NavigatePose.DISHWASHER.value.pose.orientation:
        NavigateAction([Pose(robot.get_pose().pose.position,
                             NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
    elif robot.get_pose().pose.orientation == NavigatePose.SHELF.value.pose.orientation:
        NavigateAction([Pose(robot.get_pose().pose.position,
                             NavigatePose.LONG_TABLE.value.pose.orientation)]).resolve().perform()
    elif robot.get_pose().pose.orientation == NavigatePose.LONG_TABLE.value.pose.orientation:
        NavigateAction([Pose(robot.get_pose().pose.position,
                             NavigatePose.SHELF.value.pose.orientation)]).resolve().perform()


def pickup_object(object: Object):
    global table_pose, CUTLERY
    grasp = Grasp.FRONT

    object_pose = object.pose.position
    print(object_pose)

    if object.obj_type in CUTLERY or object.obj_type == "Metalbowl":
        grasp = Grasp.TOP

    if object.obj_type == "Metalplate":
        TalkingMotion("Can you please give me the plate on the table.").perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        rospy.sleep(3)

        TalkingMotion("Grasping.").perform()

        MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
    else:
        if object.obj_type in CUTLERY: # and object.pose.position.y > table_pose + 0.125:
            object.pose.position.z = 0.71
        # change object x pose if the grasping pose is too far in the table
        # object.pose.position.y -= 0.1
        if object.obj_type == "Metalbowl":
            object.pose.position.z = 0.72
        TalkingMotion("Picking up from: " + (str(grasp)[6:]).lower()).perform()
        if grasp == Grasp.TOP:
            MoveTorsoAction([0.8]).resolve().perform()
        try_pick_up(robot, object, grasp)

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                           robot.get_pose().pose.position.y - 0.3, 0],
                                          [0, 0, 0.7, 0.7])]).resolve().perform()
    MoveTorsoAction([0]).resolve().perform()
    if object.obj_type == "Metalplate":
        MoveJointsMotion(["arm_roll_joint"], [-1.5]).perform()

    if object.obj_type in CUTLERY:
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 4.9, 0.35],
                                       NavigatePose.POPCORN_TABLE.value.pose.orientation))
        if object_found(object_desig, str(object.obj_type)):
            new_object = get_object(object_desig, str(object.obj_type))
            try_pick_up(robot, new_object, grasp)
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                                   robot.get_pose().pose.position.y - 0.3, 0],
                                                  [0, 0, 0.7, 0.7])]).resolve().perform()
            MoveTorsoAction([0]).resolve().perform()


def place_object(object: Object):
    x_y_z_pos = get_pos(str(object.obj_type).upper())

    x_pos = x_y_z_pos[0]
    y_pos = x_y_z_pos[1]
    z_pos = x_y_z_pos[2]

    if x_pos >= 2.65:
        NavigateAction([Pose([NavigatePose.DISHWASHER.value.pose.position.x + 1,
                              NavigatePose.DISHWASHER.value.pose.position.y, 0],
                             NavigatePose.DISHWASHER.value.pose.orientation)]).resolve().perform()
        NavigateAction([Pose([NavigatePose.DISHWASHER.value.pose.position.x + 1,
                              NavigatePose.DISHWASHER.value.pose.position.y - 0.5, 0],
                             NavigatePose.LONG_TABLE.value.pose.orientation)]).resolve().perform()
    else:
        NavigateAction([Pose([NavigatePose.DISHWASHER.value.pose.position.x - 0.105,
                              NavigatePose.DISHWASHER.value.pose.position.y, 0],
                             NavigatePose.DISHWASHER.value.pose.orientation)]).resolve().perform()
        NavigateAction([Pose([NavigatePose.DISHWASHER.value.pose.position.x - 0.65,
                              NavigatePose.DISHWASHER.value.pose.position.y - 0.5, 0],
                             NavigatePose.SHELF.value.pose.orientation)]).resolve().perform()

    TalkingMotion("Placing").perform()
    grasp = Grasp.FRONT

    PlaceAction(object, [Pose([x_pos, y_pos, z_pos])], [grasp], [Arms.LEFT], [False]).resolve().perform()

    # if object.obj_type == "Metalplate":
    #     PlaceGivenObjectAction(["Metalplate"], [Arms.LEFT], [Pose([x_pos, y_pos, z_pos])],
    #                            [grasp], False)
    # else:
    #     PlaceAction(object, [Pose([x_pos, y_pos, z_pos])],  [grasp], [Arms.LEFT],
    #             [False]).resolve().perform()
    # For the safety in cases where the HSR is not placing, better drop the object to not colide with the kitchen
    # drawer when moving to parkArms arm config
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()


def pickup_and_place(objects_list: list):
    NavigateAction([Pose([objects_list[0].pose.position.x, robot.get_pose().pose.position.y, 0],
                         NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
    for value in range(len(objects_list)):
        pickup_object(objects_list[value])
        # turn around
        NavigateAction([Pose(robot.get_pose().pose.position,
                             NavigatePose.DISHWASHER.value.pose.orientation)]).resolve().perform()
        if objects_list[value].obj_type in DRINKS:
            # Navigate to trash can pose
            NavigateAction([Pose([1.26, 3.58, 0], [0, 0, -1, 1])]).resolve().perform()
            throw_object(objects_list[value])
        else:
            NavigateAction([NavigatePose.DISHWASHER.value]).resolve().perform()
            place_object(objects_list[value])
        if value + 1 < len(objects_list):
            if objects_list[value].obj_type in DRINKS:
                # TODO: adjust navigate poses
                # navigate to table
                NavigateAction([Pose(NavigatePose.DISHWASHER.value.pose.position,
                                     NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
            else:
                # turn around
                NavigateAction([Pose(NavigatePose.DISHWASHER.value.pose.position,
                                     NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
                NavigateAction([Pose([objects_list[value + 1].pose.position.x,
                                      NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
                                     NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()


# TODO: implement function for droping objects in trash can
def throw_object(obj: Object):
    # TODO: adjust navigate poses
    # navigate to perceive pose
    # NavigateAction([Pose()]).resolve().perform()
    obj_desig = DetectAction(technique='all').resolve().perform()
    # try_detect(Pose([1.2, 0.257, 0.08], [0, 0, -1, 1]))
    # TODO: change "trash" name
    trash = get_object(obj_desig, "Spoon")
    # navigate to drop pose (trash can)
    # NavigateAction([Pose([1.26, 3.58, 0], [0, 0, -1, 1])]).resolve().perform()
    PlaceAction(obj, [Pose([trash.pose.position.x, trash.pose.position.y, 0.65])], [Grasp.FRONT],
                [Arms.LEFT], [False]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()


def get_pos(obj_type: str):
    """
      Getter for x and y value for placing the given object type.

      :param obj_type: type of object, which x and y pose for placing we want
      :return: the tupel of x and y value for placing that object
      """
    x_val = PlacingXPose[obj_type].value
    y_val = PlacingYPose[obj_type].value
    if obj_type == "Metalplate":
        z_val = PlacingZPose.METALPLATE.value
    else:
        z_val = PlacingZPose.OTHER.value
    return x_val, y_val, z_val


def navigate_and_detect(location_name: NavigatePose):
    """
    Navigates to a certain location and perceives.

    :param location_name: the location the robot navigates to
    :return: tupel of State and dictionary of found objects in the FOV
    """
    # TalkingMotion("Navigating").perform()

    if location_name == NavigatePose.SHELF:
        NavigateAction([NavigatePose.SHELF.value]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 3.9, 0.21], [0, 0, 0, 1]))
        objects_list = get_objects(object_desig)
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
    elif location_name == NavigatePose.POPCORN_TABLE:
        NavigateAction([Pose([NavigatePose.POPCORN_TABLE.value.pose.position.x - 0.4,
                              NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
                             NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig1 = try_detect(Pose([robot.get_pose().pose.position.x, 4.9, 0.35], [0, 0, 0.7, 0.7]))
        objects_list1 = get_objects(object_desig1)
        annotator = get_used_annotator_list(Demos.CLEAN_THE_TABLE)
        isp = ImageSendPublisher(sub_topic=annotator[0])
        isp.activate_subscriber()
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
        NavigateAction([Pose([NavigatePose.POPCORN_TABLE.value.pose.position.x + 0.4,
                              NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
                             NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig2 = try_detect(Pose([robot.get_pose().pose.position.x, 4.9, 0.35], [0, 0, 0.7, 0.7]))
        objects_list2 = get_objects(object_desig2)
        annotator = get_used_annotator_list(Demos.CLEAN_THE_TABLE)
        isp = ImageSendPublisher(sub_topic=annotator[0])
        isp.activate_subscriber()
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
        objects_list = []
        for object in objects_list1 + objects_list2:
            if object not in objects_list:
                objects_list.append(object)

        # which objects has been perceived
        if len(objects_list) == 0:
            TalkingMotion("I was not able to find any objects").perform()
        else:
            sentence = ""
            for value in range(len(objects_list)):
                if value + 1 < len(objects_list):
                    sentence += "a " + str(objects_list[value].obj_type) + ", "
                else:
                    sentence += "and a " + str(objects_list[value].obj_type)
            print(sentence)
            TalkingMotion(f"I perceived {sentence}").perform()

    else:
        raise ValueError(f'Incorrect location name: {location_name}.')

    return objects_list


def failure_handling1(sorted_obj: list):
    """
    Part 1 of the failure handling consists of perceiving a second time and pick up and placing the seen objects.

    :param sorted_obj: list of seen objects.
    :return: list of seen objects in the second round. Empty list when nothing perceived or all objects already found.
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list
    new_objects_list = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive, pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            if value.obj_type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.obj_type)
        # todo should not always navigate to middle pose. think about a case where she stands already infront
        #  of the table and didn't perceived anything.
        new_objects_list = navigate_and_detect(NavigatePose.POPCORN_TABLE)
        pickup_and_place(new_objects_list)
    return new_objects_list


def failure_handling2(sorted_obj: list, new_sorted_obj: list):
    """
    Part 2 of the failure handling, when object is not seen again, the robot is asking for human support.

    :param sorted_obj: list of already seen and transported objects
    :param new_sorted_obj: list of objects that were seen in the first part of the failure handling
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list
    # failure handling part 2
    final_sorted_obj = sorted_obj + new_sorted_obj
    if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        NavigateAction([NavigatePose.POPCORN_TABLE.value]).resolve().perform()

        print("second Check")
        for value in final_sorted_obj:
            # remove all objects that were seen and transported so far
            if value.obj_type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.obj_type)

        for val in range(len(wished_sorted_obj_list)):
            grasp = Grasp.FRONT

            print(f"next object is: {wished_sorted_obj_list[val]}")
            TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} on the table?").perform()
            rospy.sleep(4)
            TalkingMotion("Grabing.").perform()
            MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()

            ParkArmsAction([Arms.LEFT]).resolve().perform()

            NavigateAction([NavigatePose.DISHWASHER.value]).resolve().perform()

            if wished_sorted_obj_list[val] == "Metalplate" or wished_sorted_obj_list[val] == "Metalbowl":
                MoveJointsMotion(["arm_roll_joint"], [-1.5]).perform()
            x_y_z_pos = get_pos(wished_sorted_obj_list[val].upper())

            x_pos = x_y_z_pos[0]
            y_pos = x_y_z_pos[1]

            if x_pos >= 2.63:
                NavigateAction([Pose([NavigatePose.DISHWASHER.value.pose.position.x + 0.7,
                                      NavigatePose.DISHWASHER.value.pose.position.y - 0.65, 0],
                                     NavigatePose.DISHWASHER.value.pose.orientation)]).resolve().perform()
            else:
                NavigateAction([Pose([NavigatePose.DISHWASHER.value.pose.position.x - 0.75,
                                      NavigatePose.DISHWASHER.value.pose.position.y - 0.65, 0],
                                     NavigatePose.DISHWASHER.value.pose.orientation)]).resolve().perform()

            TalkingMotion("Placing").perform()
            grasp = Grasp.FRONT

            # todo add placing of plate in PlaceGivenObjAction
            if wished_sorted_obj_list[val] == "Metalplate":
                # PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"],
                # [Pose([x_pos, y_pos, 0.3])], [grasp], False).resolve().perform()
                TalkingMotion("Please take the plate and place it in the dishwasher").perform()
                rospy.sleep(2)
                TalkingMotion("Droping object now").perform()
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            else:
                PlaceGivenObjectAction([wished_sorted_obj_list[val]], [Arms.LEFT],
                                       [Pose([x_pos, y_pos, 0.3])], [grasp]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

            # navigates back if a next object exists
            if val + 1 < len(wished_sorted_obj_list):
                NavigateAction([NavigatePose.POPCORN_TABLE.value]).resolve().perform()


# Main interaction sequence with real robot
with (real_robot):
    """
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
    NavigateAction([NavigatePose.DISHWASHER_CLOSED.value]).resolve().perform()

    MoveJointsMotion(["wrist_roll_joint"], [-1.5]).perform()
    giskard.dishwasher_test(handle_name, 'sink_area_dish_washer_door_joint', door_name)
    # OpenDishwasherAction(handle_name, door_name, 0.6, 1.4, [Arms.LEFT]).resolve().perform()

    TalkingMotion("Please pull out the lower rack").perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

    NavigateAction([Pose(NavigatePose.DISHWASHER.value.pose.position,
                         NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
    """

    # detect objects
    object_desig_list = navigate_and_detect(NavigatePose.POPCORN_TABLE)

    """
    # sort objects based on distance and which we like to keep
    sorted_obj = sort_objects_euclidian(robot, object_desig_list, wished_sorted_obj_list)
    # sorted_obj = sort_objects(object_desig_list, wished_sorted_obj_list)

    # picking up and placing objects
    pickup_and_place(sorted_obj)

    new_obj_desig = failure_handling1(sorted_obj)
    failure_handling2(sorted_obj, new_obj_desig)

    rospy.loginfo("Done!")
    TalkingMotion("Done").perform()
    """
