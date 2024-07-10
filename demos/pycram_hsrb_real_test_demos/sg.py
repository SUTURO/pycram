import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal, pakerino, GraspListener
from pycram.designator import LocationDesignatorDescription
import random

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
grasp_listener = GraspListener()
talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()
lt = LocalTransformer()
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
giskardpy.clear()

shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                          [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])
table_pose_pre = Pose([2.862644998141083, 4.946512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])


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


# List of objects
objects = ["Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball", "Baseball", "Mustardbottle",
           "Jellochocolatepuddingbox", "Wineglass", "Orange", "Coffeepack", "Softball", "Metalplate",
           "Pringleschipscan", "Strawberry", "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug",
           "Abrasivesponge", "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox", "Coffeecan",
           "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan", "Gelatinebox", "Pear", "Lemon", "Banana",
           "Pottedmeatcan", "Peach", "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
           "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small", "screwdriver", "clamp", "hammer",
           "wooden_block", "Cornybox*"]

# Group objects by similarity
groups = {"Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
          "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue",
                                       "Cupgreen", "Cup_small"],
          "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
          "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                          "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                          "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                          "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan"],
          "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
          "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
          "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"],
          "Unknown": ["Unknown"]}
# 'shelf:shelf:shelf_floor_0',
links_from_shelf = ['shelf:shelf:shelf_floor_1', 'shelf:shelf:shelf_floor_2', ]

popcorn_frame = "popcorn_table:p_table:table_front_edge_center"


def find_group(obj):
    obj_lower = obj.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


def get_closet_link_to_pose(obj_pose):
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


def get_closest_pose(obj_pose, pose_list):
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
    link = None
    nearest_pose_to_group = None
    try:
        link = groups_in_shelf[group][1]
        group_pose = groups_in_shelf[group][0]
        place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                          object_desig=object)
        nearest_pose_to_group = get_closest_pose(group_pose, place_poses)
        pace_poses = []
        for link in links_from_shelf:
            place_poses.append(
                (find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                     object_desig=object), link)
            )

        # in this case its the biggest group since why not? i assume most poses are then free
        longest_group = max(place_poses, key=lambda x: len(x[0]))
        nearest_pose_to_group = longest_group[0]
        group = ...  # Assuming 'group' is defined somewhere in your code
        groups_in_shelf[group][1] = longest_group[1]  # link
        groups_in_shelf[group][0] = nearest_pose_to_group

        place_poses = []
        for link in links_from_shelf:
            place_poses.append(
                find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                    object_desig=object), link)
        # in this case its the biggest group since why not? i assume most poses are then free
        longest_group = max(place_poses, key=len)
        nearest_pose_to_group = longest_group[0]
        groups_in_shelf[group][1] = link
        groups_in_shelf[group][0] = nearest_pose_to_group
    except TypeError:



    if nearest_pose_to_group:
        world.current_bullet_world.add_vis_axis(nearest_pose_to_group)

        pose_in_shelf = lt.transform_pose(nearest_pose_to_group, kitchen.get_link_tf_frame(link))
        pose_in_shelf.pose.position.x = -0.10
        pose_in_shelf.pose.position.z += 0.06
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")
        world.current_bullet_world.add_vis_axis(adjusted_pose_in_map)

        talk.pub_now("placing to group:" + group)
        return adjusted_pose_in_map, link


previous_value = None


def monitor_func_place():
    global previous_value
    der = fts.get_last_value()
    current_value = fts.get_last_value()

    prev_force_x = previous_value.wrench.force.x
    curr_force_x = current_value.wrench.force.x
    change_in_force_x = abs(curr_force_x - prev_force_x)
    print(f"Current Force X: {curr_force_x}, Previous Force X: {prev_force_x}, Change: {change_in_force_x}")

    def calculate_dynamic_threshold(previous_force_x):
        # Placeholder for a dynamic threshold calculation based on previous values
        # This function can be enhanced to calculate a threshold based on the history of values or other logic
        return max(0.1 * abs(previous_force_x), 1.5)  # Example: 10% of the previous value or a minimum of 1.5

    if change_in_force_x >= calculate_dynamic_threshold(previous_force_x=prev_force_x):
        print("Significant change detected")

        return SensorMonitoringCondition

    return False


def placeorpark(object_name, object, grasp, talk_bool, target_location, link, pick_up_bool):
    global previous_value
    oTm = target_location
    oTm.pose.position.z += 0.02

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(link))

    if grasp == "top":
        grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
        oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
    else:
        oTb.orientation = grasp_rotation

    oTmG = lt.transform_pose(oTb, "map")

    BulletWorld.current_bullet_world.add_vis_axis(oTmG)

    # todome this mshoudl be depending on the height of the shelf tbh

    if grasp == "front":
        config_for_placing = {'arm_lift_joint': 0.20, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                              'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
    else:
        config_for_placing = {'arm_flex_joint': 0.20, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                              'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

    pakerino(config=config_for_placing)
    if pick_up_bool:
        talk.pub_now(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
    else:
        talk.pub_now(f"Placing now! {object_name.split('_')[0]} from: {grasp}")
    giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
    while not giskard_return:
        rospy.sleep(0.1)
    config_after_place = {'arm_lift_joint': 0.0}
    talk.pub_now("tracking placning now")
    previous_value = fts.get_last_value()
    try:
        plan = Code(lambda: giskardpy.test(config_after_place)) >> Monitor(monitor_func_place)
        return_plan = plan.perform()

    except Exception as e:
        print(f"Exception type: {type(e).__name__}")

    giskardpy.achieve_attached(object, tip_link="map")
    BulletWorld.robot.detach(object)
    gripper.pub_now("open")

    giskardpy.avoid_all_collisions()
    park = pakerino()
    while not park:
        print("waiting for park")
        rospy.sleep(0.1)


# noteme this is with driving! return grasped_bool, grasp, group, object, obj_id, groups_on_table
def process_pick_up_objects(talk_bool):
    # drive
    talk.pub_now("driving", True)
    move.pub_now(table_pose)
    # look
    locationtoplace = Pose([2.6868790796016738, 5.717920690528091, 0.715])
    look_pose = kitchen.get_link_pose("popcorn_table:p_table:table_center")
    # park
    perceive_conf = {'arm_lift_joint': 0.20, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    pakerino(config=perceive_conf)
    # look
    giskardpy.move_head_to_pose(locationtoplace)

    talk.pub_now("perceiving", True)
    groups_on_table = {}

    # noteme detect on table
    try:
        table_obj = DetectAction(technique='all').resolve().perform()
        first, *remaining = table_obj
        for dictionary in remaining:
            for value in dictionary.values():
                try:
                    group = find_group(value.type)
                    groups_on_table[value.name] = [value, group]
                except AttributeError:
                    pass
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        talk.pub_now("I am Done, I hope I did good!")
        return

    # noteme if groups were found
    if groups_on_table:
        groups_on_table_w_table_frame = {}
        for key, obj in groups_on_table.items():
            obj_pose = obj[0].bullet_world_object.pose
            object_raw = obj[0]
            tf_link = kitchen.get_link_tf_frame(popcorn_frame)
            oTb = lt.transform_pose(obj_pose, tf_link)

            grasp_set = None
            # if to far behind the front face were robots look on the table
            if oTb.pose.position.x >= 0.20:
                grasp_set = "top"

            # noteme this saves the group
            try:
                obj[1]
            except KeyError:
                obj[1] = "Unknown"

            groups_on_table_w_table_frame[key] = (obj[0], oTb, grasp_set, obj[1])

        # noteme sorts objects on the table with x so i can pick up the "most nearest"
        sorted_groups = sorted(groups_on_table_w_table_frame.items(), key=lambda item: item[1][1].pose.position.x)
        sorted_dict = dict(sorted_groups)

        # noteme get first items since we sorted it already
        first_key, first_value = next(iter(sorted_dict.items()))
        first_object = first_value[0]
        oTb = first_value[1]
        grasp_set = first_value[2]
        group = first_value[3]
        print(group)
        obj_id = first_object.bullet_world_object.id
        object_name = first_object.bullet_world_object.name
        object = first_object.bullet_world_object

        angle = helper.quaternion_to_angle(
            (oTb.pose.orientation.x, oTb.pose.orientation.y, oTb.pose.orientation.z, oTb.pose.orientation.w))
        object_dim = object.get_object_dimensions()
        print(f"obj dim von {object_name} {object_dim}")

        if grasp_set:
            grasp = "top"
        else:
            if object_dim[2] < 0.055:
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            elif object_dim[2] < 0.065 or angle > 40 and (object_dim[0] > 0.075 and object_dim[1] > 0.075):
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            else:
                rospy.logwarn(f"{object_name} grasp is set to front, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "front"

            if grasp == "top":
                print("pose adjusted with z")
                oTb.pose.position.z += (object_dim[2] / 10)
                if object_dim[2] < 0.02:
                    rospy.logwarn(f"I am not able to grasp the object: {object_name} please help me!")
                    oTb.pose.position.z = 0.011
            else:
                oTb.pose.position.x += 0.03

        grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
        if grasp == "top":
            grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
            oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
        else:
            oTb.orientation = grasp_rotation

        oTmG = lt.transform_pose(oTb, "map")
        after_pose = oTmG.copy()
        after_pose.pose.position.z += 0.02

        z_color = [1, 0, 1, 1]
        BulletWorld.current_bullet_world.add_vis_axis(after_pose, z_color=z_color)
        BulletWorld.current_bullet_world.add_vis_axis(oTmG)
        move.pub_now(table_pose_pre)

        if grasp == "front":
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
        else:
            config_for_placing = {'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                                  'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

        pakerino(config=config_for_placing)

        gripper.pub_now("open")
        talk.pub_now(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
        giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
        while not giskard_return:
            rospy.sleep(0.1)

        giskardpy.achieve_attached(object)
        tip_link = 'hand_gripper_tool_frame'
        BulletWorld.robot.attach(object=object, link=tip_link)
        gripper.pub_now("close")
        giskardpy.avoid_all_collisions()
        park = pakerino()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        grasped_bool = None
        if grasp_listener.check_grasp():
            talk.pub_now("Vanessas functions says i grasped a object")
            grasped_bool = True
        else:
            talk.pub_now("Vanessas functions says i  was not able to grasped a object")
            grasped_bool = False

        return grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table


# noteme  with look  return groups in shelf
def process_objects_in_shelf(talk_bool):
    # perceive shelf
    groups_in_shelf = {
        "Kitchen Utensils and Tools": None,
        "Containers and Drinkware": None,
        "Cleaning Supplies": None,
        "Packaged Food and Beverages": None,
        "Fruits": None,
        "Sports Equipment": None,
        "Miscellaneous": None,
        "Unknown": None
    }
    look_pose = kitchen.get_link_pose("shelf:shelf:shelf_floor_1")
    perceive_conf = {'arm_lift_joint': 0.20, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    pakerino(config=perceive_conf)
    # look
    giskardpy.move_head_to_pose(look_pose)

    talk.pub_now("perceiving", True)

    shelf_obj = DetectAction(technique='all').resolve().perform()
    first, *remaining = shelf_obj
    for dictionary in remaining:
        for value in dictionary.values():

            try:
                group = find_group(value.type)
                link = get_closet_link_to_pose(value.pose)
                groups_in_shelf[group] = [value.pose, link, value]

            except AttributeError:
                pass
    return groups_in_shelf


groups_in_shelf = {}


def demo(step):
    global groups_in_shelf
    with ((((real_robot)))):
        object_name = None,
        object = None,
        grasp = None,
        talk_bool = None,
        target_location = None,
        link = None
        talk_bool = True
        gripper.pub_now("close")
        pakerino()

        if step <= 1:
            talk.pub_now("driving", True)
            move.pub_now(rotated_shelf_pose, interrupt_bool=False)
            move.pub_now(shelf_pose, interrupt_bool=False)
            groups_in_shelf = process_objects_in_shelf(talk_bool)
            move.pub_now(rotated_shelf_pose, interrupt_bool=False)

        if step <= 2:
            try:
                grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table = process_pick_up_objects(
                    talk_bool)
                print(grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table)
            except TypeError:
                print("done")
                return
        if step <= 3:
            move.pub_now(rotated_shelf_pose, interrupt_bool=False)
            move.pub_now(shelf_pose, interrupt_bool=False)
            place_pose, link = find_pose_in_shelf(group, object_raw, groups_in_shelf)
            placeorpark(object.name, object, "front", talk_bool, place_pose, link, False)
            demo(2)


# previous_value = fts.get_last_value()
#
# monitor_func_place()
demo(0)
