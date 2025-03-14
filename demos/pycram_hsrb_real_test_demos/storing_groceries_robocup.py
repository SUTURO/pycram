from geometry_msgs.msg import Quaternion, Vector3
from typing_extensions import List

from demos.pycram_clean_the_table_demo.utils.misc import object_found
from pycram.datastructures.enums import WorldMode, ImageEnum
from pycram.designators.motion_designator import DoorOpenMotion
from pycram.external_interfaces import giskard

# try:
#     from demos.pycram_carry_my_luggage.utils.cml_helper import fts
# except ModuleNotFoundError:
#     pass
from pycram import helper
from pycram.designators.location_designator import find_placeable_pose
# from pycram.external_interfaces.giskard import sync_worlds
from pycram.failures import PerceptionObjectNotFound
from pycram.language import Code

from pycram.designators.action_designator import *
from pycram.process_module import real_robot, simulated_robot
# import pycram.external_interfaces.giskard_new as giskardpy
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.designators.object_designator import *
from pycram.ros_utils.robot_state_updater import KitchenStateUpdater, RobotStateUpdater
from pycram.utilities.robocup_utils import TextToSpeechPublisher, HSRBMoveGripperReal, pakerino, GraspListener, \
    ImageSwitchPublisher, StartSignalWaiter, ImageSendPublisher

from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
start_signal = StartSignalWaiter()

world = BulletWorld(WorldMode.GUI)
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_robocup_2025_1.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

lt = LocalTransformer()
navigation = PoseNavigator()
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

###############################################################################################

# Set the demo mode to either real or simulated robot. If set in simulated mode, use
# the "roslaunch giskardpy_ros giskardpy_hsr_standalone.launch".
# In addition to that, pycram.external_interfaces.giskard.sync_worlds (around line 131) has to be
# adjusted slightly. When in real robot mode, giskard cannot call "add_set_seed_*" functions,
# thus they have to commented out in the sync_worlds function. When in the simulated mode, these
# functions have to be called, in order to move the robot inside giskard.
# I have marked all lines that need to be adjusted.
# demo_mode = simulated_robot
demo_mode = real_robot

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()

annotator = get_used_annotator_list(Demos.STORING_GROCERIES)
isp = ImageSendPublisher(sub_topic=annotator[0])

# Enable or disable the talk functionality
talk_bool: bool = True

# Variables needed for the left shelf door opening
# shelf_left_door_exists = True
# start_with_left_shelf_door_open = True
# shelf_left_door_handle = "shelf_billy:shelf_billy:shelf_door_left:handle"
# shelf_left_door_joint = "shelf_billy:shelf_billy:shelf_door_left:joint"

# Variables needed for the right shelf door opening
# shelf_right_door_exists = True
# start_with_right_shelf_door_open = True
# shelf_right_door_handle = "shelf_billy:shelf_billy:shelf_door_right:handle"
# shelf_right_door_joint = "shelf_billy:shelf_billy:shelf_door_right:joint"
# shelf_door_open_state = -1.7

# Insert the correct link names here. I chose to also save the variable names seperately, in case
# I want to target a specific shelf floor: for example when perceiving the shelf initially, I currently
# look at floor 0 and 2 (and leave out shelf_floor_1), because thats enough to reliably capture the whole shelf.
shelf_floor_0 = 'shelf_billy_corridor:shelf_billy:shelf_floor_0'
shelf_floor_1 = 'shelf_billy_corridor:shelf_billy:shelf_floor_1'
shelf_floor_2 = 'shelf_billy_corridor:shelf_billy:shelf_floor_2'

# this list contains the links of the shelf floors we want to use for placing
# In this case, we will never try to place an object on shelf_floor_0
links_from_shelf = [shelf_floor_0, shelf_floor_1, shelf_floor_2]

# This link is the center link of the table where the objects are placed at the start of the challenge
pick_table_link = "dinner_table:dinner_table:table_center"

# Navigation poses. Shelf pose are where the robot drives to store the objects, table pose is where
# it drives to pick them up. The "*_drive_back" poses are used to rotate the robot before driving
# back. This is done to ensure the robots safety. If you judge it to be unnecessary, either
# remove the variables from the script, or even simpler, set them to the same pose as the normal
# Important quaternions:
# [0.0, 0.0, 0.0, 1.0]: robot is aligned with the map frame
# [0.0, 0.0, -1.0, 0.0]: robot is rotated 180 degrees from the map frame
# [0.0, 0.0, 0.707, 0.707]: robot is rotated 90 degrees counter-clockwise from the map frame
# [0.0, 0.0, -0.707, 0.707]: robot is rotated 90 degrees clockwise from the map frame
main_door_pose = Pose([1.25, 0.148, 0], [0, 0, 0, 1])
left_door_pose = Pose([3.73, 0.45, 0], [0, 0, 0.7, 0.7])
front_door_to_kitchen_pose = Pose([4.67, 0.412, 0], [0, 0, 0, 1])
left_door_to_kitchen_pose = Pose([7.65, 2.75, 0], [0, 0, -0.7, 0.7])
left_door_to_livingroom_pose = Pose([7.65, 2.75, 0], [0, 0, 0.7, 0.7])
perceive_pose = Pose([4.7, 2.9, 0.0], [0, 0, 0, 1])
shelf_pose = Pose([4.8, 3.0, 0.0], [0, 0, -0.7, 0.7])
shelf_pose_drive_back = Pose([4.8, 3.0, 0.0], [0, 0, 0, 1])
table_pose = Pose([4.6, 2.9, 0.0], [0, 0, 0, 1])
table_pose_drive_back = Pose([6.15, 3.8, 0.0], [0, 0, 1, 0])
# table_pose = Pose([8.3, -0.1, 0.0], [0.0, 0.0, -0.29, 0.956])
# table_pose_drive_back = Pose([8.3, -0.1, 0.0], [0.0, 0.0, 0.958, 0.283])

# Used to identify if an object is some form of cereal. List contains substrings that are common in cereal names
# which are used in the "if step <= 3:" block inside demo().
# We are doing this because we want to pour the cereal, not transport it
cereal_types = ["Muesli", "Cereal"]

# Specifies the types that may be used as containers for pouring.
# This is used in the "if step <= 3:" block inside demo()
container_types = ["CerealStorageContainerCornflakes", "CerealStorageContainerFruitLoops"]

# Specifies the direction to pour from ("left" or "right") and the tipping angle for
# pouring (positive angle, is inverted inside the PouringActionPerformable if needed).
direction_to_pour_from = "left"
pour_angle = 125

# Objects types where we know we want to grasp from the top. Otherwise, process_pick_up_objects will
# try to find a suitable grasp based on object dimensions etc, which may be unreliable
known_top_grasp_objects = ["Metalbowl", ObjectType.BOWL]

###############################################################################################
############# General Variables to be set after you created the new semantic map ##############

# Groups items into categories
groups = {"Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
          "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue",
                                       "Cupgreen", "Cup_small"],
          "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
          "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                          "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                          "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                          "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan",
                                          'breakfast_cereal'],
          "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
          "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
          "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"],
          "Unknown": ["Unknown"]}

# List of objects, taken from the previous version, is currently NOT being used
objects = ["Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball", "Baseball", "Mustardbottle",
           "Jellochocolatepuddingbox", "Wineglass", "Orange", "Coffeepack", "Softball", "Metalplate",
           "Pringleschipscan", "Strawberry", "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug",
           "Abrasivesponge", "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox", "Coffeecan",
           "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan", "Gelatinebox", "Pear", "Lemon", "Banana",
           "Pottedmeatcan", "Peach", "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
           "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small", "screwdriver", "clamp", "hammer",
           "wooden_block", "Cornybox*", 'breakfast_cereal']

###############################################################################################
############# Joint Configs - Probably dont need to be changed, but just in case ##############

config_for_placing = {'arm_flex_joint': 0.20, 'arm_lift_joint': 0.6, 'arm_roll_joint': 0,
                      'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }
perceive_config = {'arm_lift_joint': 0.25, 'arm_roll_joint': 1.5, 'wrist_flex_joint': -1.5, }
park_config = {'arm_flex_joint': 0, 'arm_lift_joint': 0, 'arm_roll_joint': 0,
               'wrist_flex_joint': -1.9, 'wrist_roll_joint': 0}
pickup_config = {'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                 'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

###############################################################################################

if demo_mode == real_robot:
    print("Real Robot")
    grasp_listener = GraspListener()
    img_swap = ImageSwitchPublisher()
    start_signal_waiter = StartSignalWaiter()
    fts = ForceTorqueSensor(robot_name='hsrb')
    move = PoseNavigator()
    # RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
    gripper = HSRBMoveGripperReal()
else:
    # milk1 = Object("milk1", ObjectType.MILK, "milk.stl", pose=Pose([5.4, 3.8, 0.63]))
    cereal1 = Object("cereal1", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                     pose=Pose([5.4, 4.1, 1.16]))  # Pose([5.4, 4.1, 1.16])
    bowl = Object("bowl1", ObjectType.BOWL, "bowl.stl", pose=Pose([5.4, 3.8, 0.57]))  # Pose([5.4, 3.8, 0.57])
    # [2.5, 5.717920690528091, 0.715]
    # milk2 = Object("milk2", ObjectType.MILK, "milk.stl", pose=Pose([2.6, 4.8, 0.81], [0, 0, 1, 1]))
    cereal2 = Object("cereal2", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                     pose=Pose([2, 4.8, 0.82], [0, 0, 1, 1]))
    bowl2 = Object("bowl2", ObjectType.BOWL, "bowl.stl", pose=Pose([2.3, 4.8, 0.75], [0, 0, 1, 1]))


def navigate_to(nav_pose, interrupt_bool=True):
    if demo_mode == real_robot:
        move.pub_now(nav_pose, interrupt_bool=interrupt_bool)
        robot.set_pose(nav_pose)
    elif demo_mode == simulated_robot:
        NavigateAction([nav_pose]).resolve().perform()
        giskard.sync_worlds()


def park_arms():
    if demo_mode == real_robot:
        return pakerino(config=park_config)
    elif demo_mode == simulated_robot:
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveTorsoAction([0]).resolve().perform()
        giskard.sync_worlds()
        return True

def set_joint_config(z_torso=None, config=None):
    if demo_mode == real_robot:
        return pakerino(z_torso, config)
    elif demo_mode == simulated_robot:
        return MoveJointsMotion(list(config.keys()), list(config.values())).perform()


def talk_pub(sentence=Any, talk_bool=True, wait_bool=True):
    if demo_mode == real_robot:
        return talk.pub_now(sentence, talk_bool, wait_bool)
    elif demo_mode == simulated_robot:
        return print(sentence)


def move_head(look_pose):
    if demo_mode == real_robot:
        return giskard.move_head_to_pose(look_pose)
    elif demo_mode == simulated_robot:
        return LookAtAction([look_pose]).resolve().perform()


def gripper_motion(motion):
    if demo_mode == real_robot:
        gripper_state = GripperState.OPEN if motion == "open" else GripperState.CLOSE
        MoveGripperMotion(gripper_state, Arms.LEFT).perform()
    elif demo_mode == simulated_robot:
        giskard.set_gripper_state(motion)


talk = TextToSpeechPublisher()
giskard.clear()
giskard.sync_worlds()

# if start_with_left_shelf_door_open and shelf_left_door_exists:
#     kitchen.set_joint_position(shelf_left_door_joint, shelf_door_open_state)
#     if demo_mode == simulated_robot:
#         giskard.set_joint_positions(kitchen, {shelf_left_door_joint: shelf_door_open_state})
#
# if start_with_right_shelf_door_open and shelf_right_door_exists:
#     kitchen.set_joint_position(shelf_right_door_joint, shelf_door_open_state)
#     if demo_mode == simulated_robot:
#         giskard.set_joint_positions(kitchen, {shelf_right_door_joint: shelf_door_open_state})


def multiply_quaternions(q1, q2):
    """
    Multiply two quaternions.

    Parameters:
    q1 (tuple): First quaternion (x1, y1, z1, w1).
    q2 (tuple): Second quaternion (x2, y2, z2, w2).

    Returns:
    tuple: The product of the two quaternions.
    """

    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


def find_group(obj_type):
    """
    Find the group of an object type. If the object type is not in the groups, return None.

    :param obj_type: The object type.
    :return: The group of the object type.
    """
    for group, items in groups.items():
        for item in items:
            if obj_type in item.lower() or item.lower() in obj_type:
                return group
    return None


def get_closet_link_to_pose(obj_pose):
    """
    Find the closest link to a pose. The distance is calculated based on the z position of the pose.
    This is used in the beginning of the demo, when the shelf is perceived initially to identify the groups
    inside the shelf.

    :param obj_pose: The pose of the object.
    :return: The closest link to the pose.
    """
    position_distance = 30
    nearest_link = None
    for link in links_from_shelf:
        link_pose = kitchen.get_link_pose(link)
        posez = obj_pose.position.z

        dis = abs(link_pose.pose.position.z - posez)
        if dis <= position_distance:
            position_distance = dis
            nearest_link = link
    return nearest_link


def get_closest_pose(obj_pose: Pose, pose_list: List[Pose]):
    """
    Find the closest pose to an object pose from a list of poses. The list of poses was previously generated
    by a semantic cost map. The distance is calculated based on the Euclidean distance between the positions.

    :param obj_pose: The pose of the object.
    :param pose_list: The list of poses.

    :return: The closest pose to the object pose.
    """
    position_distance = float('inf')
    nearest_pose = None

    for pose in pose_list:
        dis = ((obj_pose.pose.position.x - pose.position.x) ** 2 + (obj_pose.pose.position.y - pose.position.y) ** 2 + (
                obj_pose.pose.position.z - pose.position.z) ** 2) ** 0.5
        if dis < position_distance:
            position_distance = dis
            nearest_pose = pose

    return nearest_pose


def find_pose_in_shelf(group, object, groups_in_shelf):
    """
    Find a pose in the shelf for an object. If the group is in the shelf, find the closest pose to the group.
    If the group is not in the shelf, find the biggest group of poses and place the object on the pose that is
    furthest away from the link.

    :param group: The group of the object.
    :param object: The object.
    :param groups_in_shelf: The groups in the shelf.

    :return: The adjusted pose in the map and the link of the shelf.
    """
    try:
        link = groups_in_shelf[group][1]
        group_pose = groups_in_shelf[group][0]
        place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 25,
                                          object_desig=object, clearance_radius=0.19)
        nearest_pose_to_group = get_closest_pose(group_pose, place_poses)
        place_pose = nearest_pose_to_group
    # If the group is not in the shelf, find the biggest group of poses and place the object on the pose that is
    # furthest away from the link
    except (TypeError, KeyError):
        place_poses = []
        for link in links_from_shelf:
            place_poses.append(
                (find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 25,
                                     object_desig=object), link)
            )

        longest_group = max(place_poses, key=lambda x: len(x[0]))
        link = longest_group[1]
        link_pose = kitchen.get_link_pose(link)
        furthest_away_pose = max(longest_group[0], key=lambda x: ((x.position.x - link_pose.pose.position.x) ** 2 + (
                x.position.y - link_pose.pose.position.y) ** 2 + (x.position.z - link_pose.pose.position.z) ** 2) ** 0.5)

        place_pose = furthest_away_pose
        if group not in groups_in_shelf:
            groups_in_shelf[group] = [furthest_away_pose, longest_group[1]]

    if place_pose:
        pose_in_shelf = lt.transform_pose(place_pose, kitchen.get_link_tf_frame(link))

        # Uncomment the following lines to adjust the Z position of the objects target pose in the shelf
        # You probably only want to do this if you find a *systematic* error in the Z position of the objects in the shelf
        # a adjustment of 0.03-0.05 should probably be enough
        # pose_in_shelf.pose.position.z += 0.0
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")

        return adjusted_pose_in_map, link

# uncomment if you want to use the force torque sensor
# previous_value = None
# def monitor_func_place():
#     global previous_value
#     der = fts.get_last_value()
#     current_value = fts.get_last_value()
#
#     prev_force_x = previous_value.wrench.force.x
#     curr_force_x = current_value.wrench.force.x
#     change_in_force_x = abs(curr_force_x - prev_force_x)
#     print(f"Current Force X: {curr_force_x}, Previous Force X: {prev_force_x}, Change: {change_in_force_x}")
#
#     def calculate_dynamic_threshold(previous_force_x):
#         # Placeholder for a dynamic threshold calculation based on previous values
#         # This function can be enhanced to calculate a threshold based on the history of values or other logic
#         return max(0.1 * abs(previous_force_x), 1.5)  # Example: 10% of the previous value or a minimum of 1.5
#
#     if change_in_force_x >= calculate_dynamic_threshold(previous_force_x=prev_force_x):
#         print("Significant change detected")
#
#         return SensorMonitoringCondition
#
#     return False


def place_object(object_name, object, grasp, target_location, talk_bool):
    """
    Place an object at a target location. The object is placed at the target location with the specified grasp.

    :param object_name: The name of the object.
    :param object: The object.
    :param grasp: The grasp of the object.
    :param target_location: The target location to place the object.
    :param talk_bool: A boolean to enable or disable the talk functionality.

    :return: None
    """
    global previous_value
    object_to_map = target_location
    if demo_mode == real_robot:
        object_to_map.pose.position.z += 0.03
    grasp_rotation = RobotDescription.current_robot_description.grasps[grasp]

    ###### Transfer pose to base frame, to generalize the calculations. ######
    base_frame = RobotDescription.current_robot_description.name + "/" + RobotDescription.current_robot_description.base_link
    object_to_base = lt.transform_pose(object_to_map, base_frame)
    ########################################

    if grasp == Grasp.TOP:
        object_to_base.pose.position.z += 0.03

    # Since the pose is in the base frame, this line causes the final grasp orientation to be the same as
    # the robots current orientation. This is done because currently we cannot rely on perceptions orientation
    object_to_base.orientation = grasp_rotation

    # Defines the prepose, by shifting the pose pose 20cm towards the robotm and 5cm upwards
    object_to_base_prepose = object_to_base.copy()
    object_to_base_prepose.set_position([object_to_base.position.x - 0.2, object_to_base.position.y, object_to_base.position.z + 0.05])
    object_to_map_grasp_prepose = lt.transform_pose(object_to_base_prepose, "map")

    object_to_map_grasp = lt.transform_pose(object_to_base, "map")

    if object.obj_type == ObjectType.BOWL:
        MoveTorsoAction([0.5]).resolve().perform()
    else:
        set_joint_config(config=config_for_placing)

    talk_pub(f"Placing now! {object_name.split('_')[0]} from: {grasp}", talk_bool)

    # These two code blocks both move the gripper with the object to te target pose, the second one uses sequence goals
    # while the first one doesnt. The second one is faster, but if the grasping gets unreliable because
    # of the odom problem me and simon faced, comment out the second one and use the first one instead
    ############################
    ########## No sequence goals
    giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_prepose], demo_mode)
    while not giskard_return:
        rospy.sleep(0.1)
    giskard.update_from_giskard(robot, giskard_return)

    giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp], demo_mode)
    while not giskard_return:
        rospy.sleep(0.1)
    giskard.update_from_giskard(robot, giskard_return)
    ############################
    ########## Sequence goals
    # giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_prepose, object_to_map_grasp], demo_mode)
    # while not giskard_return:
    #     rospy.sleep(0.1)
    # giskard.update_from_giskard(robot, giskard_return)
    ##########################################################

    # config_after_place = {'arm_lift_joint': 0.0}
    # if demo_mode == real_robot:
    #     previous_value = fts.get_last_value()
    #     try:
    #         talk_pub("tracking placing now", talk_bool)
    #         plan = Code(lambda: giskard.test(config_after_place)) >> Monitor(monitor_func_place)
    #         return_plan = plan.perform()
    #         place_pose = object_to_map_grasp.copy()
    #         place_pose.set_position([object_to_map_grasp.position.x, object_to_map_grasp.position.y, object_to_map_grasp.position.z-0.1])
    #         giskard_return = giskard.achieve_place_w_fts(place_pose, RobotDescription.current_robot_description.get_arm_chain(
    #             Arms.LEFT).get_tool_frame(), 'map', GiskardStateFTS.PLACE)
    #         talk_pub("Object released", talk_bool)
    #         giskard.update_from_giskard(robot, giskard_return)
    #
    #     except Exception as e:
    #         print(f"Exception type: {type(e).__name__}")

    # giskard.achieve_detached(object, demo_mode)
    BulletWorld.robot.detach(object)
    gripper_motion("open")

    # Retract to the prepose, to avoid collisions
    giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_prepose], demo_mode)
    while not giskard_return:
        rospy.sleep(0.1)
    giskard.update_from_giskard(robot, giskard_return)

    gripper_motion("close")
    giskard.avoid_all_collisions()
    park = park_arms()
    while not park:
        print("waiting for park")
        rospy.sleep(0.1)
    if demo_mode == real_robot:
        giskard.update_from_giskard(robot, park)


def process_objects_and_pick_up(talk_bool):
    """
    Process the objects on the table and pick up the first object. The objects are processed and the first object
    is picked up based on the x position of the object.

    :param talk_bool: A boolean to enable or disable the talk functionality.

    :return: A tuple containing the grasped boolean, the grasp, the group, the object, the object id,
    the groups on the table, and the original pose.
    """
    ###############################################################################################
    ################## Currently we only drive to the table, and perceive once. But if we are unlucky
    ################## and we cannot find any objects initially, for example because the table is very large
    ################## and the objects are far away or not visible at all, then we need some way to handle this
    ################## situation. One way to do this could be to define multiple "table_pose" poses, and loop through
    ################## them during our first perception until we find objects, and then save the position that worked
    ################## If we do something like that, then you need to ensure that the "PerceptionObjectNotFound" exception
    ################## does not prematurely end the demo.
    talk_pub("driving", talk_bool)
    navigate_to(table_pose)
    # look_pose = kitchen.get_link_pose(pick_table_link)
    # look_pose = table
    look_pose = Pose([5.45, 2.9, 0.0], [0, 0, -0.7, 0.7])
    set_joint_config(config=perceive_config)
    move_head(look_pose)

    talk_pub("perceiving", talk_bool)
    groups_on_table = {}
    try:
        image_switch_publisher.pub_now(ImageEnum.SEARCH.value)
        isp.activate_subscriber()
        table_obj = DetectAction(technique='all').resolve().perform()
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
        giskard.sync_worlds()
        dictionary = table_obj
        for value in dictionary.values():
            try:
                if demo_mode == real_robot:
                    obj_lower = value.obj_type.lower()
                else:
                    obj_lower = value.obj_type.name.lower()
                group = find_group(obj_lower)
                if value.obj_type in container_types and not handled_cereal:
                    continue

                if any(cereal_type.lower() in obj_lower for cereal_type in cereal_types) and handled_cereal:
                    continue
                groups_on_table[value.name] = [value, group]
            except AttributeError:
                pass
        giskard.sync_worlds()
    # Handle this differently if we need to loop through multiple table poses
    except PerceptionObjectNotFound as e:
        talk_pub("I am Done, I hope I did good!", talk_bool)
        return
    ###########################################################################################


    if groups_on_table:
        groups_on_table_w_table_frame = {}
        for key, obj in groups_on_table.items():
            obj_pose = obj[0].world_object.pose
            object_raw = obj[0]

            base_frame = RobotDescription.current_robot_description.name + "/" + RobotDescription.current_robot_description.base_link
            object_to_base = lt.transform_pose(obj_pose, base_frame)

            grasp_set = None
            if object_raw.obj_type in known_top_grasp_objects:
                grasp_set = Grasp.TOP

            # noteme if the object is unknown, set the group to unknown
            #obj[1] = "Unknown" if obj[1] is None else obj[1]
            try:
                obj[1]
            except KeyError:
                obj[1] = "Unknown"

            groups_on_table_w_table_frame[key] = (obj[0], object_to_base, grasp_set, obj[1])

        # Sort the objects on the table based on their x position, so we can pick up the nearest one
        sorted_groups = sorted(groups_on_table_w_table_frame.items(), key=lambda item: item[1][1].pose.position.x)
        sorted_dict = dict(sorted_groups)

        # Get the first object in the sorted list
        first_key, first_value = next(iter(sorted_dict.items()))
        first_object = first_value[0]
        object_to_base = first_value[1]
        original_pose = lt.transform_pose(first_value[1], "map")
        grasp_set = first_value[2]
        group = first_value[3]
        print(f"Group: {group}")
        obj_id = first_object.world_object.id
        object_name = first_object.world_object.name
        object = first_object.world_object

        angle = helper.quaternion_to_angle(
            (object_to_base.pose.orientation.x, object_to_base.pose.orientation.y, object_to_base.pose.orientation.z, object_to_base.pose.orientation.w))
        object_dim = object.get_object_dimensions()
        print(f"obj dim von {object_name} {object_dim}")

        # What follows is some logic implemented during the previous robocup, to handle objects were we do not know
        # how to grasp them. The grasping is based on the object dimensions, and the angle of the object.
        # So far I have not touched this, and I havent had any problem with it, but i am unsure why we could not
        # just map explicit grasps to the majority of the known objects. I will leave this as is for now, but
        # you may want to think about this and maybe change it.
        if grasp_set:
            grasp = grasp_set
        else:
            if object_dim[2] < 0.055:
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = Grasp.TOP
            elif object_dim[2] < 0.065 or angle > 40 and (object_dim[0] > 0.075 and object_dim[1] > 0.075):
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = Grasp.TOP
            else:
                rospy.logwarn(f"{object_name} grasp is set to front, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = Grasp.FRONT

            if grasp == Grasp.TOP:
                print("pose adjusted with z")
                object_to_base.pose.position.z += (object_dim[2] / 10)
                if object_dim[2] < 0.02:
                    rospy.logwarn(f"I am not able to grasp the object: {object_name} please help me!")
                    object_to_base.pose.position.z = 0.011
            else:
                object_to_base.pose.position.x += 0.03

        if first_object.obj_type == ObjectType.BOWL:
            object_to_base.pose.position.z += 0.02
            object_to_base.pose.position.y += 0.05

        grasp_rotation = RobotDescription.current_robot_description.grasps[grasp]
        if grasp == Grasp.TOP:
            grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
            object_to_base.orientation = multiply_quaternions(object_to_base.pose.orientation, grasp_q)
        else:
            object_to_base.orientation = grasp_rotation

        # Prepose Calculations
        object_to_base_prepose = object_to_base.copy()
        object_to_base_prepose.set_position([object_to_base.position.x - 0.2, object_to_base.position.y, object_to_base.position.z + 0.05])
        object_to_map_grasp_prepose = lt.transform_pose(object_to_base_prepose, "map")
        object_to_map_grasp = lt.transform_pose(object_to_base, "map")

        # Lift Calculations
        object_to_base_lift = object_to_base.copy()
        object_to_base_lift.set_position([object_to_base.position.x, object_to_base.position.y, object_to_base.position.z + 0.05])
        object_to_map_grasp_lift = lt.transform_pose(object_to_base_lift, "map")

        set_joint_config(config=pickup_config)

        # These two code blocks both move the gripper to the object, the second one uses sequence goals
        # while the first one doesnt. The second one is faster, but if the grasping gets unreliable because
        # of the odom problem me and simon faced, comment out the second one and use the first one instead
        ############################
        ########## No sequence goals
        giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_prepose], demo_mode)
        while not giskard_return:
            rospy.sleep(0.1)
        giskard.update_from_giskard(robot, giskard_return)

        gripper_motion("open")
        talk_pub(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}", talk_bool)
        giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp], demo_mode)
        while not giskard_return:
            rospy.sleep(0.1)
        #########################
        ########## Sequence goals
        # gripper_motion("open")
        # talk_pub(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}", talk_bool)
        # giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_prepose, object_to_map_grasp], demo_mode)
        # while not giskard_return:
        #     rospy.sleep(0.1)
        #############################################################

        # Keep this uncommented, updates the robot, attaches the object and closes the gripper
        giskard.update_from_giskard(robot, giskard_return)
        # giskard.achieve_attached(object, demo_mode=demo_mode)
        tip_link = 'hand_gripper_tool_frame'
        BulletWorld.robot.attach(child_object=object, parent_link=tip_link)
        gripper_motion("close")

        # These two code blocks both lift and retract the gripper with the object, the second one uses sequence goals
        # while the first one doesnt. The second one is faster, but if the grasping gets unreliable because
        # of the odom problem me and simon faced, comment out the second one and use the first one instead
        ############################
        ########## No sequence goals
        giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_lift], demo_mode)
        while not giskard_return:
            rospy.sleep(0.1)
        giskard.update_from_giskard(robot, giskard_return)

        giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_prepose], demo_mode)
        while not giskard_return:
            rospy.sleep(0.1)
        giskard.update_from_giskard(robot, giskard_return)
        #########################
        ########## Sequence goals
        # giskard_return = giskard.achieve_sequence_pick_up([object_to_map_grasp_lift, object_to_map_grasp_prepose], demo_mode)
        # while not giskard_return:
        #     rospy.sleep(0.1)
        #############################################################

        giskard.update_from_giskard(robot, giskard_return)
        giskard.avoid_all_collisions()

        park = park_arms()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        if demo_mode == real_robot:
            giskard.update_from_giskard(robot, park)
        if demo_mode == real_robot:
            if grasp_listener.check_grasp():
                talk_pub("Grasped a object", talk_bool)
                grasped_bool = True
            else:
                talk_pub("I was not able to grasped a object", talk_bool)
                grasped_bool = False
        else:
            grasped_bool = True

        groups_on_table.__delitem__(object_name)
        return grasped_bool, grasp, group, first_object, obj_id, groups_on_table, original_pose
    else:
        talk_pub("I think am Done, I hope I did good!", talk_bool)
        raise TypeError


def process_objects_in_shelf(talk_bool):
    """
    Process the objects in the shelf. The objects are processed in two steps, first the objects on the first floor
    of the shelf are processed, then the objects on the second floor of the shelf are processed.

    :param talk_bool: A boolean to enable or disable the talk functionality.

    :return: A dictionary containing the groups in the shelf.
    """

    text_to_speech_publisher.pub_now("look at my screen please")
    rospy.sleep(0.5)
    image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
    TalkingMotion("look at my screen please")
    rospy.sleep(0.5)

    groups_in_shelf = {key: None for key in groups.keys()}

    look_pose = kitchen.get_link_pose(shelf_floor_0)
    set_joint_config(config=perceive_config)
    move_head(look_pose)

    talk_pub("perceiving", talk_bool=talk_bool)

    try:
        image_switch_publisher.pub_now(ImageEnum.SEARCH.value)
        isp.activate_subscriber()
        shelf_obj = DetectAction(technique='all').resolve().perform()
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
    except PerceptionObjectNotFound:
        shelf_obj = {}
    giskard.sync_worlds()
    dictionary = shelf_obj
    for value in dictionary.values():
        try:
            if demo_mode == real_robot:
                obj_lower = value.obj_type.lower()
            else:
                obj_lower = value.obj_type.name.lower()
            group = find_group(obj_lower)
            link = get_closet_link_to_pose(value.pose)
            groups_in_shelf[group] = [value.pose, link, value]

        except AttributeError:
            pass

    look_pose = kitchen.get_link_pose(shelf_floor_2)
    move_head(look_pose)

    talk_pub("perceiving", talk_bool=talk_bool)

    try:
        image_switch_publisher.pub_now(ImageEnum.SEARCH.value)
        isp.activate_subscriber()
        shelf_obj = DetectAction(technique='all').resolve().perform()
        image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
    except PerceptionObjectNotFound:
        shelf_obj = {}
    giskard.sync_worlds()
    dictionary = shelf_obj
    for value in dictionary.values():

        try:
            if demo_mode == real_robot:
                obj_lower = value.obj_type.lower()
            else:
                obj_lower = value.obj_type.name.lower()
            group = find_group(obj_lower)
            link = get_closet_link_to_pose(value.pose)
            groups_in_shelf[group] = [value.pose, link, value]

        except AttributeError:
            pass

    return groups_in_shelf


groups_in_shelf = {}
handled_cereal = False


def get_table(obj_dict: dict):
    """
    searches in a dictionary of objects for a bowl and returns it
    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :return: the found bowl or None
    """
    if len(obj_dict) == 0:
        return None
    for value in obj_dict.values():
        if value.obj_type == "Table":
            return value
    return None


def look_for_table(angle: float):
    MoveJointsMotion(["head_pan_joint"], [angle]).perform()
    obj_desig = DetectAction(technique='all').resolve().perform()
    if object_found(obj_desig, "table"):
        table = get_table(obj_desig)
        return table
    return


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


import numpy as np


def quaternion_rotate_180(q, axis):
    """
    Dreht die gegebene Quaternion q um 180 Grad um die angegebene Achse.

    :param q: Tuple oder Liste (w, x, y, z) - die ursprüngliche Quaternion
    :param axis: Tuple oder Liste (x, y, z) - die Drehachse
    :return: Tuple (w', x', y', z') - die gedrehte Quaternion
    """
    # Normiere die Achse
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)  # Sicherstellen, dass die Achse normiert ist

    # Erstelle die 180°-Dreh-Quaternion
    q_180 = (0, axis[0], axis[1], axis[2])

    # Multipliziere die Quaternions
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = q_180

    w_new = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_new = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_new = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_new = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return (w_new, x_new, y_new, z_new)


# Beispiel: Eine Quaternion um 180 Grad um die Y-Achse drehen
q = (1, 0, 0, 0)  # Identitäts-Quaternion
ergebnis = quaternion_rotate_180(q, (0, 1, 0))
print(ergebnis)


def demo(step):
    global groups_in_shelf, handled_cereal, table_pose
    with ((((demo_mode)))):
        try:
            talk_pub("push down my hand when you are ready")

            plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            print("start demo")

        # park = park_arms()
        # while not park:
        #     print("waiting for park")
        #     rospy.sleep(0.1)
        # if demo_mode == real_robot:
        #     giskard.update_from_giskard(robot, park)
        start_signal.wait_for_startsignal()
        start_pose = robot.get_pose()
        navigation.pub_fake_pose(start_pose)
        giskard.turning_left_and_back(45)

        object = None,
        grasp = None,
        talk_bool = True
        gripper_motion("close")
        # park = park_arms()
        # while not park:
        #     print("waiting for park")
        #     rospy.sleep(0.1)
        # if demo_mode == real_robot:
        #     giskard.update_from_giskard(robot, park)

        if step <= 1:
            talk_pub("driving", talk_bool)
            navigate_to(left_door_pose)
            navigate_to(shelf_pose, interrupt_bool=False)
            TalkingMotion("Can you please open the right door of the shelf?")
            text_to_speech_publisher.pub_now("Can you please open the right door of the shelf?")
            rospy.sleep(0.5)
            image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
            rospy.sleep(2)
            image_switch_publisher.pub_now(ImageEnum.HI.value)

            giskard.billy_shelf_open(shelf_pose)

            navigate_to(Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y + 0.3, 0],
                             [0, 0, -0.7, 0.7]))
            navigate_to(shelf_pose, interrupt_bool=False)
            groups_in_shelf = process_objects_in_shelf(talk_bool)
            park = park_arms()
            while not park:
                print("waiting for park")
                rospy.sleep(0.1)
            if demo_mode == real_robot:
                giskard.update_from_giskard(robot, park)
            # navigate_to(shelf_pose_drive_back, interrupt_bool=False)
            # navigate_to(table_pose, interrupt_bool=False)
            # TalkingMotion("looking for the table").perform()
            # MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            # MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            # obj_desig = DetectAction(technique='all').resolve().perform()
            # if object_found(obj_desig, "table"):
            #     table = get_table(obj_desig)
            # else:
            #     MoveJointsMotion(["head_pan_joint"], [-0.5]).perform()
            #     obj_desig = DetectAction(technique='all').resolve().perform()
            #     if object_found(obj_desig, "table"):
            #         table = get_table(obj_desig)
            #     else:
            #         MoveJointsMotion(["head_pan_joint"], [-0.9]).perform()
            #         obj_desig = DetectAction(technique='all').resolve().perform()
            #         if object_found(obj_desig, "table"):
            #             table = get_table(obj_desig)
            #         else:
            #             TalkingMotion("I was not able to find the table.").perform()
            # table_pose = table.pose

        if step <= 2:
            try:
                grasped_bool, grasp, group,  object_raw, obj_id, groups_on_table, original_pose = process_objects_and_pick_up(
                    talk_bool)
            except TypeError as e:
                print(f"done (caught error {e})")
                return

        if step <= 3:
            bowl = None
            if demo_mode == real_robot:
                obj_type = object_raw.obj_type.lower()
            else:
                obj_type = object_raw.obj_type.name.lower()
            if any(cereal_type.lower() in obj_type for cereal_type in cereal_types):
                navigate_to(table_pose)

                table_obj = DetectAction(technique='all').resolve().perform()
                bowl = next((value for value in table_obj.values() if value.obj_type in container_types), None)
            if bowl:
                handled_cereal = True
                PouringAction([bowl.pose], [Arms.LEFT], [direction_to_pour_from], [pour_angle]).resolve().perform()
                park = park_arms()
                while not park:
                    print("waiting for park")
                    rospy.sleep(0.1)
                if demo_mode == real_robot:
                    giskard.update_from_giskard(robot, park)
                navigate_to(table_pose)
                place_object(object_raw.name, object_raw.world_object, grasp, original_pose, talk_bool)

            else:
                # drive_back_orientation = quaternion_rotate_180(table.pose.orientation, [0, 0, 1])
                # table_pose_drive_back = Pose(table_pose.pose.position, drive_back_orientation)
                # navigate_to(table_pose_drive_back, interrupt_bool=False)
                # navigate_to(table_pose, interrupt_bool=False)
                navigate_to(shelf_pose, interrupt_bool=False)
                giskard.sync_worlds()
                place_pose, link = find_pose_in_shelf(group, object_raw, groups_in_shelf)
                place_object(object_raw.name, object_raw.world_object, grasp, place_pose, talk_bool)
            demo(2)

        print()


# previous_value = fts.get_last_value()
#
# monitor_func_place()



demo(0)


