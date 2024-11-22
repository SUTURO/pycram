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
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
lt = LocalTransformer()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])

shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                          [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])
table_pose_pre = Pose([2.862644998141083, 4.946512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]), color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])


milk_desig = BelieveObject(names=["milk"])
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



def calculate_z_offsets(links_from_shelf):
    """
    Calculate the Z offsets between each link in links_from_shelf and return a dictionary mapping each link
    to the Z height difference to the next link.
    """
    z_offsets = {}

    # Get the pose (including Z position) for each link
    link_poses = {link: kitchen.get_link_pose(link) for link in links_from_shelf}

    # Iterate through the links and calculate Z offsets
    for i, link in enumerate(links_from_shelf[:-1]):  # Skip the last link
        current_pose = link_poses[link]
        next_pose = link_poses[links_from_shelf[i + 1]]
        current_z = current_pose.position.z  # Adjust according to the structure of Pose
        next_z = next_pose.position.z  # Adjust according to the structure of Pose
        z_offset = next_z - current_z
        z_offsets[link] = z_offset

    return z_offsets

def get_z_height_to_next_link(link, z_offsets):
    """
    Return the Z height difference from the given link to the next link.
    """
    return z_offsets.get(link, None)


#noteme if you want to be able to run this in simulation you will have to do get.aabb without bullet world obj
def find_pose_in_shelf(group, object, groups_in_shelf):
    link = None
    nearest_pose_to_group = None
    try:
        link = groups_in_shelf[group][1]
        group_pose = groups_in_shelf[group][0]
        place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                          object_desig=object)
        nearest_pose_to_group = get_closest_pose(group_pose, place_poses)

    except (TypeError, KeyError):
        place_poses = []
        for link in links_from_shelf:
            place_poses.append(
                (find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                     object_desig=object), link)
            )

        # in this case it's the biggest group since why not? i assume most poses are then free
        longest_group = max(place_poses, key=lambda x: len(x[0]))
        nearest_pose_to_group = longest_group[0][0]
        link = longest_group[1]
        if group not in groups_in_shelf:
            groups_in_shelf[group] = [ nearest_pose_to_group, longest_group[1] ]


    if nearest_pose_to_group:
        z_offsets = calculate_z_offsets(links_from_shelf)
        z_height_to_next = get_z_height_to_next_link(link, z_offsets)

        pose_in_shelf = lt.transform_pose(nearest_pose_to_group, kitchen.get_link_tf_frame(link))
        pose_in_shelf.pose.position.x = -0.10
        if z_height_to_next:
            pose_in_shelf.pose.position.z = (z_height_to_next/2)-0.01
        else:
            pose_in_shelf.pose.position.z += 0.03
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")
        world.current_bullet_world.add_vis_axis(adjusted_pose_in_map)

        return adjusted_pose_in_map, link





groups_in_shelf = {}


def demo(step):
    global groups_in_shelf
    place_pose, link = find_pose_in_shelf("Fruits", milk, groups_in_shelf)
    milk.set_pose(place_pose)
    place_pose, link = find_pose_in_shelf("Fruits", cereal, groups_in_shelf)
    cereal.set_pose(place_pose)
    place_pose, link = find_pose_in_shelf("Kitchen Utensils and Tools", bowl, groups_in_shelf)
    bowl.set_pose(place_pose)




demo(0)
