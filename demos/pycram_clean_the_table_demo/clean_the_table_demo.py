import rospy
from geometry_msgs.msg import PoseStamped
from typing_extensions import Optional

from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from pycram.external_interfaces import giskard
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.failures import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.utilities.robocup_utils import ImageSendPublisher, StartSignalWaiter
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

# TODO: write a function for deffering between NavigateAction and move.pub_now() + adjust calls in the code

# Initialization of needed things for the demo
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()

# start signal for door opening
start_signal = StartSignalWaiter()

# Pose Navigator to use instead of NavigateAction if needed
navigation = PoseNavigator()

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon", "Knife"]
# wished_sorted_obj_list = ["Metalplate"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# if the dishwasher is opened at the start of the demo or not
opened = False

# Should the start of the demo be from outside or not
from_outside = False

# placing on upper rack
with_upper_rack = False

# name of the dishwasher handle, dishwasher door and dishwasher hinge
handle_name = "iai_kitchen/sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"
hinge_name = 'sink_area_dish_washer_door_joint'

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

# Believe kitchen object
apart_desig = BelieveObject(names=["kitchen"])


class NavigatePose(Enum):
    """
    Enum for Navigate Poses to easier change poses if needed
    """
    DISHWASHER_CLOSED = Pose([2.753, -2.0, 0], [0, 0, -1, 1])
    DISHWASHER_LEFT = Pose([3.753, -2.35, 0], [0, 0, 1, 0])
    DISHWASHER_RIGHT = Pose([1.93, -2.35, 0], [0, 0, 0, 1])
    DISHWASHER = Pose([2.95, -1.85, 0], [0, 0, -1, 1])
    SHELF = Pose([4.53, 3.95, 0], [0, 0, 0, 1])
    POPCORN_TABLE = Pose([1.98, 4, 0], [0, 0, 0.7, 0.7])
    LONG_TABLE = Pose([1.73, 0.8, 0], [0, 0, 1, 0])
    TRASH_CAN = Pose([1.1, 3.5, 0], [0, 0, -1, 1])


class PlacingXPose(Enum):
    """
    Enum for differentiate the x pose for placing in the dishwasher
    """
    CUTLERY = 2.67
    SPOON = 2.67
    FORK = 2.67
    PLASTICKNIFE = 2.67
    KNIFE = 2.67
    METALBOWL = 3.0
    METALMUG = 2.98
    METALPLATE = 2.82


class PlacingYPose(Enum):
    """
    Enum for differentiate the y pose for placing in the dishwasher
    """
    CUTLERY = -2.57
    SPOON = -2.57
    FORK = -2.57
    PLASTICKNIFE = -2.57
    KNIFE = -2.57
    METALBOWL = -2.66
    METALMUG = -2.59
    METALPLATE = -2.65


class PlacingZPose(Enum):
    """
    Enum for differentiate the z pose for placing in the dishwasher
    """
    METALPLATE = 0.52
    OTHER = 0.47
    UPPER = 0.77


def pickup_object(object: Object):
    """
    Pick up an object with failure handling and prepare for moving
    :param object: The object that has to be picked up
    """
    # setting grasp to FRONT as default
    grasp = Grasp.FRONT

    if object.obj_type in CUTLERY or object.obj_type == "Metalbowl":
        grasp = Grasp.TOP

    # If the object is the Plate the robot ask a human to give it the plate
    # (pick up of a Plate is not possible until now)
    if object.obj_type == "Metalplate":
        TalkingMotion("Can you please give me the plate on the table.").perform()
        rospy.sleep(1)
        try:
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            TalkingMotion("Put the plate in my gripper please").perform()
            rospy.sleep(1)
            TalkingMotion("Push down my hand when everything is ready").perform()

            # let the robot wait until somebody pushes its gripper when the plate is in there
            plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            rospy.sleep(1.5)
            TalkingMotion("Grasping.").perform()
            MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
    else:
        """
        # The height of the table where the robot pick up the objects
        table_height = 0.71
        if object.obj_type in CUTLERY:
            # set the z position of cutlery as the table height, because Perception does not always return the
            # right z position
            object.pose.position.z = table_height
        if object.obj_type == "Metalbowl":
            # Here it is similar to the previous one too, but we add 0.01 more to the table height for better pickup
            object.pose.position.z = table_height + 0.01
        """
        TalkingMotion(f"Picking up the {object.obj_type} from: " + (str(grasp)[6:]).lower()).perform()
        # To pick up Objects from TOP the robot needs more space to do that so the Torso should be higher than
        # pickup from FRONT
        if grasp == Grasp.TOP:
            MoveTorsoAction([0.8]).resolve().perform()
        else:
            MoveTorsoAction([0.4]).resolve().perform()
        # Pick up with Failure Handling
        try_pick_up_c(robot, object, grasp, NavigatePose.POPCORN_TABLE.value)

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    # TODO: test this one if that does not work good comment the old one again
    step_back(robot, 0.3)
    # NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
    #                                        robot.get_pose().pose.position.y - 0.3, 0],
    #                                       [0, 0, 0.7, 0.7])]).resolve().perform()

    # Failure handling for Cutlery after picking up, because we do not have Force-Torque thresholds
    # for picking up cutlery (the robot looks on the table to see if the object were picked up)
    if object.obj_type in CUTLERY:
        MoveTorsoAction([0.12]).resolve().perform()
        # TODO: test this one if that does not work good comment the old one again
        object_desig = try_detect_with_tilting(-0.2)
        # object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 4.9, 0.35],
        # NavigatePose.POPCORN_TABLE.value.pose.orientation))
        if object_found(object_desig, str(object.obj_type)):
            new_object = get_object(object_desig, str(object.obj_type))
            try_pick_up_c(robot, new_object, grasp, NavigatePose.POPCORN_TABLE.value)
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            # TODO: test this one if that does not work good comment the old one again
            step_back(robot, 0.3)
            # NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
            #                                        robot.get_pose().pose.position.y - 0.3, 0],
            #                                       [0, 0, 0.7, 0.7])]).resolve().perform()
            MoveTorsoAction([0]).resolve().perform()
    else:
        MoveTorsoAction([0]).resolve().perform()

    # Move the gripper to the right oto avoid collision with kitchen counter while moving
    if object.obj_type == "Metalplate" or object.obj_type == "Metalbowl":
        MoveJointsMotion(["arm_roll_joint"], [-1.5]).perform()


def place_object(object: Object):
    """
    Navigate to dishwasher and place an object in there
    :param object: The object that has to be placed in the dishwasher
    """
    # Get placing pose from the enums depending on the object type
    x_y_z_pos = get_pos(str(object.obj_type).upper())

    x_pos = x_y_z_pos[0]
    y_pos = x_y_z_pos[1]
    z_pos = x_y_z_pos[2]

    # Depending on the placing pose place the object from the left or the front of the dishwasher
    navigate_to(NavigatePose.DISHWASHER.value)
    # NavigateAction([NavigatePose.DISHWASHER.value]).resolve().perform()
    if x_pos >= 2.9:
        navigate_to(NavigatePose.DISHWASHER_LEFT.value)
        # NavigateAction([NavigatePose.DISHWASHER_LEFT.value]).resolve().perform()

    TalkingMotion("Placing").perform()
    grasp = Grasp.FRONT

    MoveTorsoAction([0.2]).resolve().perform()
    if object.obj_type == "Metalplate":
        # For the Plate use PlaceGivenObjectAction, because the plate was given to the robot and not picked up
        PlaceGivenObjectAction(["Metalplate"], [Arms.LEFT], [Pose([x_pos, y_pos, z_pos])], [grasp], [False])
    else:
        PlaceAction(object, [Pose([x_pos, y_pos, z_pos])], [grasp], [Arms.LEFT], [False]).resolve().perform()

    # For the safety in cases where the HSR is not placing, better drop the object to not collide with the kitchen
    # drawer when moving to parkArms arm config
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
    park_arms_and_move_torso(0)


def pickup_and_place(objects_list: list):
    """
    Loops over all object found to pickup and place them.
    :param objects_list: The list of the sorted found objects on the table
    """
    # If any objects were found, move to the first object to pick it up
    if len(objects_list) != 0:
        navigate_to(Pose([objects_list[0].pose.position.x, NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
                         NavigatePose.POPCORN_TABLE.value.pose.orientation))
        # NavigateAction([Pose([objects_list[0].pose.position.x, NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
        #                      NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
    for value in range(len(objects_list)):
        pickup_object(objects_list[value])
        # Turn around
        navigate_to(Pose(robot.get_pose().pose.position,
                         NavigatePose.DISHWASHER.value.pose.orientation))
        # NavigateAction([Pose(robot.get_pose().pose.position,
        #                      NavigatePose.DISHWASHER.value.pose.orientation)]).resolve().perform()
        # If the picked up object is a drink throw it in the trash can, otherwise place it in the dishwasher
        if objects_list[value].obj_type in DRINKS:
            navigate_to(NavigatePose.TRASH_CAN.value)
            # NavigateAction([NavigatePose.TRASH_CAN.value]).resolve().perform()
            throw_object(objects_list[value])
        else:
            navigate_to(NavigatePose.DISHWASHER.value)
            # NavigateAction([NavigatePose.DISHWASHER.value]).resolve().perform()
            place_object(objects_list[value])
        # Navigate to table if there are any objects remaining for pickup
        if value + 1 < len(objects_list):
            # Turn around
            navigate_to(Pose(robot.get_pose().pose.position,
                             NavigatePose.POPCORN_TABLE.value.pose.orientation))
            # NavigateAction([Pose(robot.get_pose().pose.position,
            #                      NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
            # Navigate to table
            navigate_to(Pose([objects_list[value + 1].pose.position.x,
                              NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
                             NavigatePose.POPCORN_TABLE.value.pose.orientation))
            # NavigateAction([Pose([objects_list[value + 1].pose.position.x,
            #                       NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
            #                      NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()


def throw_object(obj: Object):
    """
    Navigate to trash can and throw an object in there
    :param obj: The object that has to be thrown in the trash can
    """
    obj_desig = try_detect_with_tilting(-0.8)
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    real_trash_can = get_object(obj_desig, "Trashbin")
    PlaceAction(obj, [Pose([real_trash_can.pose.position.x, real_trash_can.pose.position.y, 0.6])], [Grasp.FRONT],
                [Arms.LEFT], [False]).resolve().perform()
    park_arms_and_move_torso(0)


def get_pos(obj_type: str):
    """
      Getter for x, y and z values for placing the given object type.

      :param obj_type: Type of the object we want to have the placing pose for
      :return: A tupel of x, y and z values for placing that object
      """
    x_val = PlacingXPose[obj_type].value
    y_val = PlacingYPose[obj_type].value
    if obj_type == "Metalplate":
        z_val = PlacingZPose.METALPLATE.value
    elif (obj_type == "Metalbowl" or obj_type == "Metalmug") and with_upper_rack:
        z_val = PlacingZPose.UPPER.value
    else:
        z_val = PlacingZPose.OTHER.value
    return x_val, y_val, z_val


def navigate_and_detect(location_name: NavigatePose):
    """
    Navigates to a certain location and perceives the objects there.

    :param location_name: the location the robot navigates to
    :return: A list of found objects
    """
    # Annotator for the showing the images on the screen of the robot
    annotator = get_used_annotator_list(Demos.CLEAN_THE_TABLE)
    isp = ImageSendPublisher(sub_topic=annotator[0])

    rospy.sleep(0.5)
    image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
    rospy.sleep(0.5)

    if location_name == NavigatePose.SHELF:
        navigate_to(NavigatePose.SHELF.value)
        # NavigateAction([NavigatePose.SHELF.value]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig = try_detect_with_tilting(-0.4)
        # object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 3.9, 0.21], [0, 0, 0, 1]))
        objects_list = get_objects(object_desig)
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
    elif location_name == NavigatePose.POPCORN_TABLE:
        # To perceive all objects perceive the left side and the right side of the table
        navigate_to(Pose([NavigatePose.POPCORN_TABLE.value.pose.position.x - 0.4,
                          NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
                         NavigatePose.POPCORN_TABLE.value.pose.orientation))
        # NavigateAction([Pose([NavigatePose.POPCORN_TABLE.value.pose.position.x - 0.4,
        #                       NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
        #                      NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        image_switch_publisher.pub_now(ImageEnum.SEARCH.value)
        isp.activate_subscriber()
        object_desig1 = try_detect_with_tilting(-0.2)
        # object_desig1 = try_detect(Pose([robot.get_pose().pose.position.x, 4.9, 0.35], [0, 0, 0.7, 0.7]))
        objects_list1 = get_objects(object_desig1)
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
        navigate_to(Pose([NavigatePose.POPCORN_TABLE.value.pose.position.x + 0.4,
                          NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
                         NavigatePose.POPCORN_TABLE.value.pose.orientation))
        # NavigateAction([Pose([NavigatePose.POPCORN_TABLE.value.pose.position.x + 0.4,
        #                       NavigatePose.POPCORN_TABLE.value.pose.position.y, 0],
        #                      NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        image_switch_publisher.pub_now(ImageEnum.SEARCH.value)
        isp.activate_subscriber()
        object_desig2 = try_detect_with_tilting(-0.2)
        # object_desig2 = try_detect(Pose([robot.get_pose().pose.position.x, 4.9, 0.35], [0, 0, 0.7, 0.7]))
        objects_list2 = get_objects(object_desig2)
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
        objects_list = []
        for object in objects_list1 + objects_list2:
            if object not in objects_list:
                objects_list.append(object)

        # The robot says which objects has been perceived
        if len(objects_list) == 0:
            TalkingMotion("I was not able to find any objects").perform()
        else:
            sentence = ""
            for value in range(len(objects_list)):
                if len(objects_list) == 1:
                    sentence += "a " + str(objects_list[value].obj_type)
                elif value + 1 < len(objects_list):
                    sentence += "a " + str(objects_list[value].obj_type) + ", "
                else:
                    sentence += "and a " + str(objects_list[value].obj_type)
            print(sentence)
            TalkingMotion(f"I perceived {sentence}").perform()

    else:
        raise ValueError(f'Incorrect location name: {location_name}.')

    return objects_list


def navigate_to(pose: PoseStamped):
    """
    Lets the robot navigates to a pose using NavigateAction or move.pub_now()

    :param pose: The pose the robot navigates to.
    """
    with_action = False
    if with_action:
        NavigateAction([pose]).resolve().perform()
    else:
        move.pub_now(pose)


def failure_handling1(sorted_obj: list):
    """
    Part 1 of the failure handling consists of perceiving a second time and pick up and placing the seen objects.

    :param sorted_obj: list of seen objects.
    :return: list of seen objects in the second round. Empty list when nothing perceived or all objects already found.
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list
    new_objects_list = []

    # if not all needed objects found, the robot will perceive, pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far
            if value.obj_type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.obj_type)
        # todo should not always navigate to middle pose. think about a case where she stands already infront
        #  of the table and didn't perceived anything.

        # If no object was removed from the wished object list, then there were no object transported
        # >> no need to perceive again, because it were perceived exactly before being here
        # >> move directly to next case
        new_objects_list = []
        if len(wished_sorted_obj_list) != LEN_WISHED_SORTED_OBJ_LIST:
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
        navigate_to(NavigatePose.POPCORN_TABLE.value)
        # NavigateAction([NavigatePose.POPCORN_TABLE.value]).resolve().perform()

        print("second Check")
        for value in final_sorted_obj:
            # remove all objects that were seen and transported so far
            if value.obj_type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.obj_type)

        for val in range(len(wished_sorted_obj_list)):
            TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} on the table?").perform()
            rospy.sleep(1)
            try:
                TalkingMotion("push down my hand when I should grasp the object").perform()

                plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                rospy.sleep(3)
                TalkingMotion("Grabing.").perform()
                MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()

            ParkArmsAction([Arms.LEFT]).resolve().perform()

            navigate_to(NavigatePose.DISHWASHER.value)
            # NavigateAction([NavigatePose.DISHWASHER.value]).resolve().perform()

            if wished_sorted_obj_list[val] == "Metalplate" or wished_sorted_obj_list[val] == "Metalbowl":
                MoveJointsMotion(["arm_roll_joint"], [-1.5]).perform()
            x_y_z_pos = get_pos(wished_sorted_obj_list[val].upper())

            x_pos = x_y_z_pos[0]
            y_pos = x_y_z_pos[1]
            z_pos = x_y_z_pos[2]

            if x_pos >= 2.65:
                navigate_to(NavigatePose.DISHWASHER_LEFT.value)
                # NavigateAction([NavigatePose.DISHWASHER_LEFT.value]).resolve().perform()
            else:
                navigate_to(NavigatePose.DISHWASHER_RIGHT.value)
                # NavigateAction([NavigatePose.DISHWASHER_RIGHT.value]).resolve().perform()

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
                                       [Pose([x_pos, y_pos, z_pos])], [grasp], False).resolve().perform()
            park_arms_and_move_torso(0)

            # navigates back if a next object exists
            if val + 1 < len(wished_sorted_obj_list):
                # turn around
                navigate_to(Pose(robot.get_pose().pose.position,
                                 NavigatePose.POPCORN_TABLE.value.pose.orientation))
                # NavigateAction([Pose(robot.get_pose().pose.position,
                #                      NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()
                navigate_to(NavigatePose.POPCORN_TABLE.value)
                # NavigateAction([NavigatePose.POPCORN_TABLE.value]).resolve().perform()


def park_arms_and_move_torso(height: float):
    """
    Park arms and move the torso to a specific height
    :param height: the height, the robot should move its torso to
    """
    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
    MoveTorsoAction([height]).resolve().perform()


# Main interaction sequence with real robot
with (real_robot):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").perform()

    # Wait for door to open to go inside the room
    if from_outside:
        start_signal.wait_for_startsignal()
        start_pose = robot.get_pose()
        navigation.pub_fake_pose(start_pose)
        giskard.turning_left_and_back(45)

    park_arms_and_move_torso(0)

    navigate_to(NavigatePose.DISHWASHER_CLOSED.value)
    # NavigateAction([NavigatePose.DISHWASHER_CLOSED.value]).resolve().perform()

    if not opened:
        TalkingMotion("I will open the dishwasher now").perform()
        MoveJointsMotion(["wrist_roll_joint"], [-1.5]).perform()
        MoveJointsMotion(["arm_roll_joint"], [0]).perform()
        OpenDishwasherAction(handle_name, hinge_name, door_name, [Arms.LEFT]).resolve().perform()
        park_arms_and_move_torso(0)
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

    if with_upper_rack:
        TalkingMotion("Please pull out the lower and upper rack").perform()
    else:
        TalkingMotion("Please pull out the lower rack").perform()

    navigate_to(Pose(NavigatePose.DISHWASHER.value.pose.position,
                     NavigatePose.POPCORN_TABLE.value.pose.orientation))
    # NavigateAction([Pose(NavigatePose.DISHWASHER.value.pose.position,
    #                      NavigatePose.POPCORN_TABLE.value.pose.orientation)]).resolve().perform()

    # detect objects
    object_desig_list = navigate_and_detect(NavigatePose.POPCORN_TABLE)

    # sort objects and keep only wished objects for the demo
    sorted_obj = sort_objects(object_desig_list, wished_sorted_obj_list)

    # picking up and placing objects
    pickup_and_place(sorted_obj)

    # TODO: Adjust Failure handling and add new cases

    # Maybe failure handling using list
    # after pickup, placing, throwing or even pouring make a list of picked up objects, plced objects etc.

    new_obj_list = failure_handling1(sorted_obj)
    failure_handling2(sorted_obj, new_obj_list)

    rospy.loginfo("Done!")
    TalkingMotion("Done").perform()
