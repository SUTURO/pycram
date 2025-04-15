import rospy
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
from demos.pycram_serve_breakfast_demo.utils.misc import try_pick_up, try_detect_with_tilting
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.utilities.robocup_utils import ImageSendPublisher, StartSignalWaiter
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()

fts = ForceTorqueSensor(robot_name='hsrb')
start_signal = StartSignalWaiter()
navigation = PoseNavigator()


# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

DRINKS = ["AppleJuice", "OatMilk", "MezzoMixBottle", "MilkPackBerch", "IceTeaFuze", "SpriteCan",
          "TeaBagBoxBad", "ColaBottle"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalmug", "Metalbowl", "Fork", "Spoon", "Metalplate"]

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

table = "left"
# TODO: change orientation
pose = Pose([8.35, 0.1, 0.0], [0.0, 0.0, 0.29, 0.959])
if table == "right":
    pose = Pose([8.35, 0.1, 0.0], [0.0, 0.0, -0.29, 0.956])

class NavigatePose(Enum):
    MAIN_DOOR = Pose([1.25, 0.148, 0], [0, 0, 0, 1])
    FRONT_DOOR_TO_KITCHEN = Pose([3.73, 0.45, 0], [0, 0, 0, 1])
    DISHWASHER_CLOSED = Pose([7.2, -0.78, 0], [0, 0, -1, 1])
    DISHWASHER_LEFT = Pose([8.2, -1.15, 0], [0, 0, -1, 0])
    DISHWASHER_FRONT = Pose([7.23, -0.22, 0], [0, 0, -0.7, 0.7])
    KITCHEN_TABLE = pose
    TRASH_CAN = Pose([6.5, -0.6, 0], [0, 0, -1, 0])


class PlacingXPose(Enum):
    """
    Differentiate the x pose for placing
    """
    CUTLERY = 7.22
    SPOON = 7.22
    FORK = 7.22
    PLASTICKNIFE = 7.22
    KNIFE = 7.22
    METALBOWL = 7.42
    METALMUG = 7.3
    METALPLATE = 7.3


class PlacingYPose(Enum):
    """
    Differentiate the y pose for placing
    """
    CUTLERY = -1.12
    SPOON = -1.12
    FORK = -1.12
    PLASTICKNIFE = -1.12
    KNIFE = -1.12
    METALBOWL = -1.35
    METALMUG = -1.35
    METALPLATE = -1.45


class PlacingZPose(Enum):
    """
    Differentiate the z pose for placing
    """
    METALPLATE = 0.6
    OTHER = 0.5


def table_pose(table: str):
    if table == "left":
        return Pose()



def pickup_object(object: Object):
    global table_pose, CUTLERY
    grasp = Grasp.FRONT

    object_pose = object.pose.position
    print(object_pose)

    if object.obj_type in CUTLERY or object.obj_type == "Metalbowl":
        grasp = Grasp.TOP

    if object.obj_type == "Metalplate":
        TalkingMotion("Can you please give me the plate on the table?").perform()
        TalkingMotion("Put it in my gripper.").perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        rospy.sleep(5)

        TalkingMotion("Grasping.").perform()

        MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
    else:
        if object.obj_type in CUTLERY:
            object.pose.position.z = 0.69
        # change object x pose if the grasping pose is too far in the table
        # object.pose.position.y -= 0.1
        if object.obj_type == "Metalbowl":
            object.pose.position.z = 0.7
        TalkingMotion("Picking up from: " + (str(grasp)[6:]).lower()).perform()
        if grasp == Grasp.TOP:
            MoveTorsoAction([0.8]).resolve().perform()
        else:
            MoveTorsoAction([0.4]).resolve().perform()
        try_pick_up_robocup(robot, object, grasp)

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction([Pose([robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y + 0.3, 0],
                         robot.get_pose().pose.orientation)]).resolve().perform()
    MoveTorsoAction([0]).resolve().perform()

    if object.obj_type in CUTLERY:
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig = try_detect(Pose([9.1, -0.55, 0.683], NavigatePose.KITCHEN_TABLE.value.pose.orientation))
        if object_found(object_desig, str(object.obj_type)):
            new_object = get_object(object_desig, str(object.obj_type))
            try_pick_up_robocup(robot, new_object, grasp)
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            NavigateAction([Pose([robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y + 0.3, 0],
                                 robot.get_pose().pose.orientation)]).resolve().perform()
            MoveTorsoAction([0]).resolve().perform()


def place_object(object: Object):
    x_y_z_pos = get_pos(str(object.obj_type).upper())

    x_pos = x_y_z_pos[0]
    y_pos = x_y_z_pos[1]
    z_pos = x_y_z_pos[2]

    if x_pos >= 7.3:
        NavigateAction([NavigatePose.DISHWASHER_LEFT.value]).resolve().perform()
    else:
        NavigateAction([NavigatePose.DISHWASHER_FRONT.value]).resolve().perform()

    TalkingMotion("Placing").perform()
    grasp = Grasp.FRONT

    PlaceAction(object, [Pose([x_pos, y_pos, z_pos])], [grasp], [Arms.LEFT], [False]).resolve().perform()
    # For the safety in cases where the HSR is not placing, better drop the object to not colide with the kitchen
    # drawer when moving to parkArms arm config
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()


def pickup_and_place(objects_list: list):
    NavigateAction([NavigatePose.KITCHEN_TABLE.value]).resolve().perform()
    for value in range(len(objects_list)):
        pickup_object(objects_list[value])
        # turn around
        NavigateAction([Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y, 0],
                             [0, 0, -1, 0])]).resolve().perform()
        if objects_list[value].obj_type in DRINKS:
            # Navigate to trash can pose
            NavigateAction([NavigatePose.TRASH_CAN.value]).resolve().perform()
            throw_object(objects_list[value])
        else:
            place_object(objects_list[value])
        if value + 1 < len(objects_list):
            # turn around
            NavigateAction([Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y, 0],
                                 [0, 0, 0, 1])]).resolve().perform()
            # navigate to table
            NavigateAction([NavigatePose.KITCHEN_TABLE.value]).resolve().perform()


def throw_object(obj: Object):
    obj_desig = try_detect_with_tilting(-0.8)
    real_trash_can = get_object(obj_desig, "Trashbin")
    PlaceAction(obj, [Pose([real_trash_can.pose.position.x, real_trash_can.pose.position.y, 0.6])], [Grasp.FRONT],
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

    TalkingMotion("driving").perform()
    NavigateAction([NavigatePose.KITCHEN_TABLE.value]).resolve().perform()
    TalkingMotion("Can you remove the chairs from the table?").perform()
    rospy.sleep(3)
    TalkingMotion("look at my screen please").perform()
    MoveTorsoAction([0.12]).resolve().perform()
    image_switch_publisher.pub_now(ImageEnum.SEARCH.value)
    isp.activate_subscriber()
    object_desig1 = try_detect_with_tilting(-0.2)
    # TODO: test try_detect pose

    # object_desig1 = try_detect(Pose([9.1, -0.55, 0.683], NavigatePose.KITCHEN_TABLE.value.pose.orientation))
    objects_list = get_objects(object_desig1)
    image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
    # rospy.sleep(0.5)
    # image_switch_publisher.pub_now(ImageEnum.HI.value)

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

    return objects_list


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False

# Main interaction sequence with real robot
with (real_robot):
    try:
        TalkingMotion("push down my hand when you are ready").perform()

        plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        print("start demo")

    start_signal.wait_for_startsignal()
    start_pose = robot.get_pose()
    navigation.pub_fake_pose(start_pose)
    giskard.turning_left_and_back(45)

    # ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
    TalkingMotion("driving").perform()
    NavigateAction([NavigatePose.FRONT_DOOR_TO_KITCHEN.value]).resolve().perform()
    NavigateAction([NavigatePose.DISHWASHER_CLOSED.value]).resolve().perform()

    annotator = get_used_annotator_list(Demos.CLEAN_THE_TABLE)
    isp = ImageSendPublisher(sub_topic=annotator[0])

    opened = False
    if not opened:
        TalkingMotion("look at my screen please").perform()
        rospy.sleep(4)
        image_switch_publisher.pub_now(ImageEnum.DISHWASHER_OPENING.value)
        TalkingMotion("Can you open the dishwasher slightly? as shown on my screen.").perform()
        TalkingMotion("I will try to open the dishwasher now.").perform()
        rospy.sleep(4)

        # open dishwasher door
        # MoveJointsMotion(["wrist_roll_joint"], [-1.5]).perform()
        giskard.dishwasher_test(handle_name, 'sink_area_dish_washer_door_joint', door_name)
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        TalkingMotion("Please pull out the lower rack").perform()
    else:
        TalkingMotion("Can you open the dishwasher and pull out the lower rack?").perform()

    # detect objects
    object_desig_list = navigate_and_detect(NavigatePose.KITCHEN_TABLE)

    # sort objects based on distance and which we like to keep
    # sorted_obj = sort_objects_euclidian(robot, object_desig_list, wished_sorted_obj_list)
    sorted_obj = sort_objects(object_desig_list)

    # picking up and placing objects
    pickup_and_place(sorted_obj)

    rospy.loginfo("Done!")
    TalkingMotion("Done").perform()
