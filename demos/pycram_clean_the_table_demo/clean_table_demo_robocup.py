import time
from enum import Enum
import rospy.core
from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
# from pycram.external_interfaces.knowrob import get_table_pose
from pycram.utilities.robocup_utils import StartSignalWaiter
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher
from pycram.language import Monitor, Code

# Create an instance of the StartSignalWaiter
start_signal_waiter = StartSignalWaiter()
text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
move = PoseNavigator()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Fork", "Spoon", "Metalmug"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# todo not needed right now
# ------------------------
center_segment = 4.76
right_segment = 5.2
left_segment = 4.1
objects_on_right_side = []
objects_on_left_side = []
# ------------------------


# x pose of the end of the couch table
table_pose = 4.84
picking_up_cutlery_height = 0.77
pickup_location_name = "dinner_table"
dishwasher_location_name = "dishwasher"
placing_location_name_left = "dishwasher_left"
placing_location_name_right = "dishwasher_right"
dishwasher_placing_pos = "dishwasher_front"

# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"

# Intermediate positions for a safer navigation
move_to_the_middle_dishwasher_pose = Pose([8.9, 4.72, 0], [0, 0, 0, 1])
move_to_middle_table_pose = Pose([9.1, 4.72, 0], [0,0,1,0])
# navigating position to the correct room
move_to_living_room = Pose([5.8, 0.26, 0], [0, 0, 0, 1])
move_in_the_middle_of_living_room = Pose([8.4, 0.86, 0], [0, 0, 0, 1])
move_to_kitchen = Pose([9.2, 3.08, 0], [0, 0, 0.7, 0.7])

perceiving_y_pos = 4.76
look_at_pose = Pose([8, 4.76, 0.25])

goal_pose = None

# ForceTorqueSensor for recognizing push on the hand
fts = ForceTorqueSensor(robot_name='hsrb')

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_clean_v1.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()

# Wait for the start signal
start_signal_waiter.wait_for_startsignal()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")


def pickup_and_place_objects(sorted_obj: list):
    """
    For picking up and placing the objects in the given object designator list.

    :param sorted_obj: the distance sorted list of seen object designators.
    """
    global move_to_the_middle_dishwasher_pose, placing_location_name_right, dishwasher_placing_pos, pickup_location_name, CUTLERY

    for value in range(len(sorted_obj)):
        print("first navigation")

        # define grasping pose
        grasp = "front"
        if sorted_obj[value].type in CUTLERY:
            sorted_obj[value].type = "Cutlery"
            # todo height of the table for picking up
            sorted_obj[value].pose.position.z = picking_up_cutlery_height

        if sorted_obj[value].type == "Metalbowl":
            sorted_obj[value].pose.position.z -= 0.02

        if sorted_obj[value].type in ["Metalbowl", "Cutlery"]:
            grasp = "top"

        if sorted_obj[value].type == "Metalplate":
            text_to_speech_publisher.pub_now("Can you please give me the plate on the table.")
            image_switch_publisher.pub_now(5)
            MoveGripperMotion("open", "left").resolve().perform()
            text_to_speech_publisher.pub_now("Push down my hand, when I should grasp")
            try:
                plan = Code(lambda: rospy.sleep(1)) * 99999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                rospy.logwarn("Close Gripper")
                text_to_speech_publisher.pub_now("Grasping.")  # Todo could be deleted if demo to long
                MoveGripperMotion(motion="close", gripper="left").resolve().perform()

        else:
            text_to_speech_publisher.pub_now("Picking up with: " + sorted_obj[value].type)
            image_switch_publisher.pub_now(7)
            if grasp == "front":
                config_for_placing = {'arm_flex_joint': -0.16, 'arm_lift_joint': 0.50, 'arm_roll_joint': -0.0145,
                                      'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
            else:
                config_for_placing = {'arm_flex_joint': -1.6, 'arm_lift_joint': 0.67, 'arm_roll_joint': 0,
                                      'wrist_flex_joint': -1.4, 'wrist_roll_joint': 0}

            giskardpy.avoid_all_collisions()
            giskardpy.achieve_joint_goal(config_for_placing)

            try_pick_up(robot, sorted_obj[value], grasp)
            image_switch_publisher.pub_now(0)

        # placing the object
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        move.pub_now(move_to_the_middle_dishwasher_pose)

        # if sorted_obj[value].type == "Metalplate" or sorted_obj[value].type == "Metalbowl":
        #     MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()

        placing_pose = get_placing_pos(sorted_obj[value].type)
        # todo: silverware tray must be on the right side of the dishwasher
        if sorted_obj[value].type in ["Metalbowl", "Metalmug"]:
            navigate_to(placing_location_name_right)
        else:
            navigate_to(dishwasher_placing_pos)

        text_to_speech_publisher.pub_now("Placing")
        image_switch_publisher.pub_now(8)
        grasp = "front"

        PlaceAction(sorted_obj[value], ["left"], [grasp], [placing_pose]).resolve().perform()
        image_switch_publisher.pub_now(0)
        # For the safety in cases where the HSR is not placing, better drop the object to not colide with the kitchen drawer when moving to parkArms arm config
        MoveGripperMotion("open", "left").resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()

        # navigates back if a next object exists
        if value + 1 < len(sorted_obj):
            navigate_to(pickup_location_name, sorted_obj[value + 1].pose.position.y)


def monitor_func():
    der: WrenchStamped() = fts.get_last_value()
    print(abs(der.wrench.force.x))
    if abs(der.wrench.force.x) > 10.30:
        print(abs(der.wrench.force.x))
        print(abs(der.wrench.torque.x))
        return SensorMonitoringCondition
    return False


def get_placing_pos(obj):
    lt = LocalTransformer()
    dishwasher_main_name = "sink_area_dish_washer_main"
    link = apartment.get_link_tf_frame(dishwasher_main_name)

    world.current_bullet_world.add_vis_axis(apartment.get_link_pose(dishwasher_main_name))
    if obj == "Cutlery":
        # z = 0.48
        dishwasher = Pose([0.6050592811085371, 0.26473268332425715, 0.026000003814697303],
                          [0, 0, -0.7073882378922517, 0.7068252124052276])
    elif obj == "Metalbowl":
        dishwasher = Pose([0.4646978333378744, -0.18915569940846222, 0.026000003814697303],
                          [0, 0, -0.7073882378922517, 0.7068252124052276])
    elif obj == "Metalmug":
        dishwasher = Pose([0.4447296892064927, -0.14913978732403788, 0.026000003814697303],
                          [0, 0, -0.7073882378922517, 0.7068252124052276])
    elif obj == "Metalplate":
        # z = 0.55
        dishwasher = Pose([0.5447137327423908, -0.16921940480574493, 0.09600000381469737],
                          [0, 0, -0.7073882378922517, 0.7068252124052276])
    elif obj == "Dishwashertab":
        # todo: Werte ändern
        dishwasher = Pose([0.5447137327423908, -0.16921940480574493, 0.06600000381469734],
                          [0, 0, -0.7073882378922517, 0.7068252124052276])
    elif obj == "check":
        dishwasher = Pose([0.56, 0, 0.06600000381469734, 0.03],
                          [0, 0, -0.7073882378922517, 0.7068252124052276])

    dishwasher.header.frame_id = link  # auskommentieren, wenn 1) verwendet
    newp = lt.transform_pose(dishwasher, "map")  # link statt map wenn 1) verwendet. map wenn 2) verwendet
    print(newp)
    world.current_bullet_world.add_vis_axis(newp)
    res = Pose([newp.pose.position.x, newp.pose.position.y, newp.pose.position.z],
               [newp.pose.orientation.x, newp.pose.orientation.y, newp.pose.orientation.z, newp.pose.orientation.w])
    return res


def check_position():
    global goal_pose
    current_pose = robot.get_pose().pose.position
    euclidean_dist = math.sqrt(pow((goal_pose.pose.position.x - current_pose.x), 2) +
                               pow((goal_pose.pose.position.y - current_pose.y), 2))
    if euclidean_dist < 0.08:
        print("return true")
        return True
    print("return false")
    return False


def navigate_to(location_name: str, y: Optional[float] = None):
    """
    Navigates to the couch table or to the dishwasher on different sides.

    :param y: y pose to navigate to the couch table for picking up objects
    :param location_name: defines the name of the location to move to
    """
    global goal_pose
    if location_name == pickup_location_name and y is not None:
        goal_pose = Pose([8.8, y, 0], [0, 0, 1, 0])
        if sorted_obj_len == 1:
            goal_pose = Pose([8.9, y, 0], [0, 0, 1, 0])
            #move.pub_now(Pose([8.9, y, 0], [0, 0, 1, 0]))

        move.pub_now(move_to_middle_table_pose)
        while not check_position():
            move.pub_now(goal_pose)
    # todo left is not needed right now
    elif location_name == placing_location_name_left:
        print("left")
        goal_pose = Pose([9.8, 5.1, 0], [0, 0, -0.7, 0.7])
        move.pub_now(move_to_the_middle_dishwasher_pose)
        while not check_position():
            move.pub_now(goal_pose)

    elif location_name == placing_location_name_right:
        goal_pose = Pose([9.75, 4.18, 0], [0, 0, 0.7, 0.7])
        move.pub_now(move_to_the_middle_dishwasher_pose)
        while not check_position():
            move.pub_now(goal_pose)

    elif location_name == dishwasher_placing_pos:
        goal_pose = Pose([9.18, 4.72, 0], [0, 0, 0, 1])
        move.pub_now(move_to_the_middle_dishwasher_pose)
        while not check_position():
            move.pub_now(goal_pose)
    else:
        rospy.logerr(f"Failure. Y-Value must be set for the navigateAction to the {pickup_location_name}")


def navigate_and_detect():
    """
    Navigates to the couch table and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    global sorted_obj_len
    text_to_speech_publisher.pub_now("Navigating")

    navigate_to(pickup_location_name, perceiving_y_pos)
    MoveTorsoAction([0.2]).resolve().perform()
    MoveJointsMotion(["arm_roll_joint"], [1.5]).resolve().perform()
    LookAtAction(targets=[look_at_pose]).resolve().perform()

    text_to_speech_publisher.pub_now("Perceiving")
    image_switch_publisher.pub_now(10)
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()

        if len(object_desig) == 0:
            sorted_obj_len = 1
        else:
            sorted_obj_len = 0
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def failure_handling1(sorted_obj: list):
    """
    Part 1 of the failure handling consists of perceiving a second time and pick up and placing the seen objects.

    :param sorted_obj: list of seen objects.
    :return: list of seen objects in the second round. Empty list when nothing perceived or all objects already found.
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list, move_to_the_middle_table_pose
    new_sorted_obj = []
    found_plate = False
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive, pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            if value.type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.type)
        if safe_metalplate is not None:
            wished_sorted_obj_list.remove(safe_metalplate.type)

        new_object_desig = navigate_and_detect()
        new_sorted_obj = sort_objects(robot, new_object_desig, wished_sorted_obj_list)

        if len(new_sorted_obj) + len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
            for obj in sorted_obj:
                if obj.type == "Metalplate":
                    new_sorted_obj.remove(obj)
                    break
        else:
            if safe_metalplate is not None:
                new_sorted_obj.append(safe_metalplate)
            # metalplate must have been seen otherwise we wouldn't be in here

        pickup_and_place_objects(new_sorted_obj)
    return new_sorted_obj


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
        navigate_to(pickup_location_name, perceiving_y_pos)
        print("second Check")

        for value in new_sorted_obj: # todo changed from final_sorted_obj to new_sorted_obj
            # remove all objects that were seen and transported so far
            if value.type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.type)

        for val in range(len(wished_sorted_obj_list)):
            grasp = "front"

            print(f"next object is: {wished_sorted_obj_list[val]}")
            text_to_speech_publisher.pub_now(f"Can you please give me the {wished_sorted_obj_list[val]} on the table?")
            image_switch_publisher.pub_now(5)
            text_to_speech_publisher.pub_now("Push down my hand, when I should grasp")
            try:
                plan = Code(lambda: rospy.sleep(1)) * 99999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                rospy.logwarn("Close Gripper")
                text_to_speech_publisher.pub_now("Grasping.")  # Todo could be deleted if demo to long
                MoveGripperMotion(motion="close", gripper="left").resolve().perform()

            image_switch_publisher.pub_now(0)

            ParkArmsAction([Arms.LEFT]).resolve().perform()

            placing_pose = get_placing_pos(sorted_obj[val].type)
            move.pub_now(move_to_the_middle_dishwasher_pose)
            # if wished_sorted_obj_list[val] == "Metalplate" or wished_sorted_obj_list[val] == "Metalbowl":
            #     MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()

            # todo: silverware tray must be on the right side of the dishwasher
            if sorted_obj[val].type in ["Metalbowl", "Metalmug"]:
                navigate_to(placing_location_name_right)
            else:
                navigate_to(dishwasher_placing_pos)

            text_to_speech_publisher.pub_now("Placing")
            image_switch_publisher.pub_now(8)
            if wished_sorted_obj_list[val] == "Metalplate":
                PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"],
                                    [placing_pose], [grasp], False).resolve().perform()
            else:

                PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"],
                                    [placing_pose], [grasp]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            image_switch_publisher.pub_now(0)
            # navigates back if a next object exists
            if val + 1 < len(wished_sorted_obj_list):
                navigate_to(pickup_location_name, perceiving_y_pos)


# Main interaction sequence with real robot
with real_robot:
    rospy.loginfo("Starting demo")
    text_to_speech_publisher.pub_now("Starting demo Clean The Table")
    sorted_obj_len = 0
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    # navigate to dishwasher
    # todo: add functionality to drive into the room
    image_switch_publisher.pub_now(37) # todo which number??
    text_to_speech_publisher.pub_now("Please remove the chairs")
    text_to_speech_publisher.pub_now("between the table and the dishwasher")
    move.pub_now(move_to_living_room)
    move.pub_now(move_in_the_middle_of_living_room)
    move.pub_now(move_to_kitchen)

    MoveJointsMotion(["wrist_roll_joint"], [-1.5]).resolve().perform()
    navigate_to(dishwasher_placing_pos)

    image_switch_publisher.pub_now(2)
    text_to_speech_publisher.pub_now("Opening")
    OpenDishwasherAction(handle_name, door_name, 0.6, 1.4, ["left"]).resolve().perform()

    text_to_speech_publisher.pub_now("Please pull out the lower rack")
    image_switch_publisher.pub_now(0) #todo: which number
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveGripperMotion("open", "left").resolve().perform()

    image_switch_publisher.pub_now(0)
    # todo add this, when it is working
    # ----------------------------------
    # sorted_obj = segmentation_detection()
    # ----------------------------------
    # detect objects
    object_desig = navigate_and_detect()
    image_switch_publisher.pub_now(0)

    # sort objects based on distance and which we like to keep
    sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)
    safe_metalplate = None
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        for obj in sorted_obj:
            if obj.type == "Metalplate":
                safe_metalplate = obj
                sorted_obj.remove(obj)


    # picking up and placing objects
    pickup_and_place_objects(sorted_obj)

    new_obj_desig = failure_handling1(sorted_obj)
    failure_handling2(sorted_obj, new_obj_desig)

    move.pub_now(Pose([8.9, 4.72, 0], [0, 0, 0, 1]))
    MoveGripperMotion("close", "left").resolve().perform()
    config_for_pushing_rack = {'arm_flex_joint': -1.6, 'arm_lift_joint': 0.05, 'arm_roll_joint': 0,
                               'wrist_flex_joint': -1.4, 'wrist_roll_joint': -0.4}
    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config_for_pushing_rack)

    MoveJointsMotion(["wrist_flex_joint"], [0.0]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    rospy.loginfo("Done!")
    text_to_speech_publisher.pub_now("Done")
    image_switch_publisher.pub_now(3)





# todo new functionalities for segmentation perceiving
# --------------------------------------------------------------------------------------------
def sort_objects_new(robot: BulletWorldObject, obj_dict: dict, wished_sorted_obj_list: list):
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
    robot_pose = robot.get_pose()
    # calculate euclidian distance for all found object in a dictionary
    for value in obj_dict.values():
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


def new_nav_and_detect(segment: float):
    """
    Navigates to the couch table and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    global sorted_obj_len
    text_to_speech_publisher.pub_now("Navigating")

    look_at_pose = Pose([7.9, segment, 0.25])
    goal_pose = Pose([8.8, segment, 0], [0, 0, 1, 0])

    move.pub_now(goal_pose)
    MoveTorsoAction([0.4]).resolve().perform()
    MoveJointsMotion(["arm_roll_joint"], [1.5]).resolve().perform()
    LookAtAction(targets=[look_at_pose]).resolve().perform()

    text_to_speech_publisher.pub_now("Perceiving")
    image_switch_publisher.pub_now(10)
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        # object_desig = DetectAction(technique='region', state = pickup_location_name).resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def segmentation_detection():
    global objects_on_left_side, objects_on_right_side
    object_desig = new_nav_and_detect(center_segment)
    image_switch_publisher.pub_now(0)
    first, *remaining_center = object_desig

    # normal sort_objects needed
    sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)
    for obj in sorted_obj:
        if obj.pose.position.y > right_segment:
            objects_on_right_side.append(obj.type)

        elif obj.pose.position.y < left_segment:
            objects_on_left_side.append(obj.type)

    if objects_on_right_side:
        text_to_speech_publisher.pub_now("Perceive again on the right side")
        object_desig = new_nav_and_detect(right_segment + 0.2)
        first, *remaining_right = object_desig

    if objects_on_left_side:
        text_to_speech_publisher.pub_now("Perceive again on the left side")
        object_desig = new_nav_and_detect(left_segment)
        first, *remaining_left = object_desig

    combined_center = {}
    for dictionary_list in remaining_center:
        for val in dictionary_list:
            combined_center.update(val)

    if objects_on_right_side:
        for dictionary_list in remaining_right:
            for val in dictionary_list:
                combined_center.update(val)

    if objects_on_left_side:
        for dictionary_list in remaining_left:
            for val in dictionary_list:
                combined_center.update(val)

    sorted_obj = sort_objects_new(robot, remaining_center, wished_sorted_obj_list)
    return sorted_obj


# --------------------------------------------------------------------------------------------

def _pose_to_pose_stamped(pose: Pose) -> PoseStamped:
    """
    Transforms a PyCRAM pose to a PoseStamped message, this is necessary since Giskard NEEDS a PoseStamped message
    otherwise it will crash.

    :param pose: PyCRAM pose that should be converted
    :return: An equivalent PoseStamped message
    """
    ps = PoseStamped()
    ps.pose = pose.pose
    ps.header = pose.header

    return ps
