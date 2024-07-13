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
wished_sorted_obj_list = ["Metalplate", "Metalbowl","Fork", "Spoon", "Metalmug"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the couch table
table_pose = 4.84
picking_up_cutlery_height = 0.43
pickup_location_name = "couch_table"
placing_location_name = "dishwasher"
placing_location_name_left = "dishwasher_left"
placing_location_name_right = "dishwasher_right"

# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"
# dishwasher_main_name = "sink_area_dish_washer_main"

# Intermediate positions for a safer navigation
move_to_the_middle_table_pose = [2.2, 1.98, 0]
move_to_the_middle_dishwasher_pose = [2.2, -0.1, 0]

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
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_clean_v1.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()

# Wait for the start signal
#start_signal_waiter.wait_for_startsignal()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")


def pickup_and_place_objects(sorted_obj: list):
    """
    For picking up and placing the objects in the given object designator list.

    :param sorted_obj: the distance sorted list of seen object designators.
    """
    global move_to_the_middle_table_pose, move_to_the_middle_dishwasher_pose, table_pose, CUTLERY

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
                plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                rospy.logwarn("Close Gripper")
                text_to_speech_publisher.pub_now("Grasping.")  # Todo could be deleted if demo to long
                MoveGripperMotion(motion="close", gripper="left").resolve().perform()

        else:
            # change object x pose if the grasping pose is too far in the table
            # if sorted_obj[value].type == "Cutlery" and sorted_obj[value].pose.position.x > table_pose + 0.125:
            #     sorted_obj[value].pose.position.x -= 0.08

            text_to_speech_publisher.pub_now("Picking up with: " + grasp)
            image_switch_publisher.pub_now(7)
            if grasp == "top":
                config_for_placing = {'arm_flex_joint': -1.6, 'arm_lift_joint': 0.6, 'arm_roll_joint': 0,
                  'wrist_flex_joint': -1.4, 'wrist_roll_joint': -0.4}
                # {'arm_flex_joint': -1.6, 'arm_lift_joint': 1.15, 'arm_roll_joint': 1.6,
                #  'wrist_flex_joint': -1.4, 'wrist_roll_joint': -0.4
                giskardpy.avoid_all_collisions()
                giskardpy.achieve_joint_goal(config_for_placing)
            else:
                print("not from top")
            try_pick_up(robot, sorted_obj[value], grasp)
            image_switch_publisher.pub_now(0)

        # placing the object
        ParkArmsAction([Arms.LEFT]).resolve().perform()

        # todo check if parallel is working
        if sorted_obj[value].type == "Metalplate" or sorted_obj[value].type == "Metalbowl":
            move.pub_now(Pose(move_to_the_middle_table_pose, [0, 0, 1, 0]))
            MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()

        placing_pose = get_placing_pos(sorted_obj[value].type)
        # todo: silverware tray must be on the right side of the dishwasher
        if placing_pose.position.x >= get_placing_pos("check").position.x:
            navigate_to(placing_location_name_left)
        else:
            navigate_to(placing_location_name_right)

        text_to_speech_publisher.pub_now("Placing")
        image_switch_publisher.pub_now(8)
        grasp = "front"

        PlaceAction(sorted_obj[value], ["left"], [grasp],[placing_pose]).resolve().perform()
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
    res = Pose([newp.pose.position.x,newp.pose.position.y, newp.pose.position.z],[newp.pose.orientation.x, newp.pose.orientation.y, newp.pose.orientation.z, newp.pose.orientation.w])
    return res



def check_position():
    global goal_pose
    current_pose = robot.get_pose().pose.position
    euclidean_dist = math.sqrt(pow((goal_pose.pose.position.x - current_pose.x), 2) +
                               pow((goal_pose.pose.position.y - current_pose.y), 2))
    if euclidean_dist < 0.1:
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
        goal_pose = Pose([4.0, y, 0], [0, 0, 0, 1])
        if sorted_obj_len == 1:
             move.pub_now(Pose(move_to_the_middle_table_pose, [0, 0, 1, 1]))
        while not check_position():
            move.pub_now(goal_pose)
    elif location_name == placing_location_name_left:
        print("left")
        goal_pose = Pose([3.5, -1.32, 0], [0, 0, 1, 0])
        move.pub_now(Pose(move_to_the_middle_table_pose, [0, 0, 1, 0]))
        move.pub_now(Pose(move_to_the_middle_dishwasher_pose, [0, 0, -1, 1]))
        while not check_position():
            move.pub_now(goal_pose)

    elif location_name == placing_location_name_right:
        goal_pose = Pose([1.95, -1.48, 0], [0, 0, 0, 1])
        move.pub_now(Pose(move_to_the_middle_table_pose, [0, 0, 1, 0]))
        move.pub_now(Pose(move_to_the_middle_dishwasher_pose, [0, 0, -1, 1]))
        while not check_position():
            move.pub_now(goal_pose)
    elif location_name == placing_location_name:
        goal_pose = Pose([2.55, -0.9, 0], [0, 0, -1, 1])
        MoveJointsMotion(["wrist_roll_joint"], [-1.5]).resolve().perform()
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

    navigate_to(pickup_location_name, 2.45)  # 1.6
    MoveTorsoAction([0.1]).resolve().perform()
    MoveJointsMotion(["arm_roll_joint"], [1.5]).resolve().perform()
    LookAtAction(targets=[Pose([5.0, 2.45, 0.15])]).resolve().perform()
    #plan = move_up | look_at
    # couch table
    #plan.perform()
    text_to_speech_publisher.pub_now("Perceiving")
    image_switch_publisher.pub_now(10)
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
        if len(object_desig) > 0:
            sorted_obj_len = 1
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
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive, pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            if value.type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.type)
        # todo should not always navigate to middle pose. think about a case where she stands already infront of the table and didn't perceived anything.
        new_object_desig = navigate_and_detect()
        new_sorted_obj = sort_objects(robot, new_object_desig, wished_sorted_obj_list)
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
        navigate_to(pickup_location_name, 2.45)
        print("second Check")

        for value in final_sorted_obj:
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
                plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                rospy.logwarn("Close Gripper")
                text_to_speech_publisher.pub_now("Grasping.")  # Todo could be deleted if demo to long
                MoveGripperMotion(motion="close", gripper="left").resolve().perform()

            image_switch_publisher.pub_now(0)

            ParkArmsAction([Arms.LEFT]).resolve().perform()

            placing_pose = get_placing_pos(sorted_obj[value].type)
            # deleted a navigate

            if wished_sorted_obj_list[val] == "Metalplate" or wished_sorted_obj_list[val] == "Metalbowl":
                move.pub_now(Pose(move_to_the_middle_table_pose, [0, 0, 1, 0]))
                MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()

                # todo: silverware tray must be on the right side of the dishwasher
                if placing_pose.position.x >= get_placing_pos("check").position.x:
                    navigate_to(placing_location_name_left)
                else:
                    navigate_to(placing_location_name_right)

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
                navigate_to(pickup_location_name, 2.45)


def excecute_plan(exec_type: str, monitor_function: Optional = None, task1: Optional = None, task2: Optional = None, task3: Optional = None):
    #todo check if task can be none without error
    plan = None
    if exec_type == "parallel":
         plan = task1 | task2 | task3
    elif exec_type == "sequential":
         plan = task1 + task2 + task3
    elif exec_type == "monitor":
         plan = task1 >> Monitor(monitor_function)
    plan.perform()


# Main interaction sequence with real robot
with ((real_robot)):
    rospy.loginfo("Starting demo")
    text_to_speech_publisher.pub_now("Starting demo")
    sorted_obj_len = 0
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    #navigate_to(placing_location_name)
    # #
    # # # todo removed turn arm
    #image_switch_publisher.pub_now(2)
    #OpenDishwasherAction(handle_name, door_name, 0.6, 1.4, ["left"]).resolve().perform()
    # #
    #text_to_speech_publisher.pub_now("Please pull out the lower rack")
    # # todo can talk be integrated
    #park = ParkArmsAction([Arms.LEFT]).resolve().perform()
    #open_gripper = MoveGripperMotion("open", "left").resolve().perform()
    # plan = park | open_gripper
    # plan.perform()
    # todo example code
    # excecute_plan("parallel", task1=ParkArmsAction([Arms.LEFT]), task2=MoveGripperMotion("open", "left"), task3=text_to_speech_publisher.pub_now("Please pull out the lower rack"))
    # excecute_plan("parallel", task1=ParkArmsAction([Arms.LEFT]), task2=MoveGripperMotion("close", "left"))
    # detect objects
    object_desig = navigate_and_detect()
    image_switch_publisher.pub_now(0)

    # sort objects based on distance and which we like to keep
    sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)

    # picking up and placing objects
    pickup_and_place_objects(sorted_obj)

    new_obj_desig = failure_handling1(sorted_obj)
    failure_handling2(sorted_obj, new_obj_desig)

    rospy.loginfo("Done!")
    text_to_speech_publisher.pub_now("Done")
    image_switch_publisher.pub_now(3)
