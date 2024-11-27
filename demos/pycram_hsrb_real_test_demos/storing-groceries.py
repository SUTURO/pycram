import numpy as np
import rospy
from pycram.designators.location_designator import find_placeable_pose

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot, semi_real_robot
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter
import utils.misc as utils

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15_reduced_corrected_v4.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()


robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
#RobotStateUpdater("/tf", "/giskard_joint_states")
#KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.4868790796016738, 5.717920690528091, 0.815]), color=[1, 0, 0, 1])

giskardpy.init_giskard_interface()



# List of objects
objects = [
    "Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball",
    "Baseball", "Mustardbottle", "Jellochocolatepuddingbox", "Wineglass", "Orange",
    "Coffeepack", "Softball", "Metalplate", "Pringleschipscan", "Strawberry",
    "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug", "Abrasivesponge",
    "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox",
    "Coffeecan", "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan",
    "Gelatinebox", "Pear", "Lemon", "Banana", "Pottedmeatcan", "Peach",
    "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
    "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small",
    "screwdriver", "clamp", "hammer", "wooden_block", "Cornybox*"
]

# Group objects by similarity
groups = {
    "Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
    "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue", "Cupgreen",
                                 "Cup_small"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
    "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                    "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                    "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                    "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"],
    "Unknown": ["Unknown"]
}
#######################################################################################################################
links_from_shelf = [
    'shelf:shelf:shelf_floor_0',
    'shelf:shelf:shelf_floor_1',
    'shelf:shelf:shelf_floor_2',
]


#######################################################################################################################


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
        dis = ((obj_pose.pose.position.x - pose.position.x) ** 2 +
               (obj_pose.pose.position.y - pose.position.y) ** 2 +
               (obj_pose.pose.position.z - pose.position.z) ** 2) ** 0.5
        if dis < position_distance:
            position_distance = dis
            nearest_pose = pose

    return nearest_pose


#######################################################################################################################

shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                          [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])


def demo(step):
    with semi_real_robot:
        # Wait for the start signal
        utils.pakerino()

        if step <= 0:
            start_signal_waiter.wait_for_startsignal()

            # continue with the rest of the script
            rospy.loginfo("Start signal received, now proceeding with tasks.")

        # move to shelf
        if step <= 1:
            rospy.logerr("driving")
            NavigateAction([shelf_pose]).resolve().perform()
            talk.pub_now("driving", True)
            rospy.logerr("1")
            #move.query_pose_nav(rotated_shelf_pose)
            #move.query_pose_nav(shelf_pose)

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
        groups_on_table = {}

        if step <= 2:
            look_pose = kitchen.get_link_pose("shelf:shelf:shelf_floor_1")
            # giskardpy.move_head_to_pose(look_pose)
            LookAtAction(targets=[look_pose]).resolve().perform()  # 0.18
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

        if step <= 3:

            rospy.logerr("driving")
            talk.pub_now("driving", True)
            NavigateAction([table_pose]).resolve().perform()
            #move.query_pose_nav(rotated_shelf_pose)
            #move.query_pose_nav(table_pose)
            look_pose = kitchen.get_link_pose("popcorn_table:p_table:table_center")
            look_pose.pose.position.x += 0.5
            world.current_bullet_world.add_vis_axis(look_pose)
            LookAtAction(targets=[look_pose]).resolve().perform()  # 0.18

            table_obj = DetectAction(technique='all').resolve().perform()
            first, *remaining = table_obj
            for dictionary in remaining:
                for value in dictionary.values():
                    try:
                        group = find_group(value.type)
                        groups_on_table[value.name] = [value, group]

                    except AttributeError:
                        pass

        if step <= 4:
            for obj in groups_on_table.values():
                utils.pickerino(obj[0], "front", "left", talk)
                utils.pakerino()
                rospy.logerr("driving")
                talk.pub_now("driving", True)

                NavigateAction([shelf_pose]).resolve().perform()
               # move.query_pose_nav(shelf_pose)

                try:
                    obj[1]
                except KeyError:
                    obj[1] = "Unknown"

                if groups_in_shelf[obj[1]] not in [None, "Unknown"]:
                    try:
                        link = groups_in_shelf[obj[1]][1]
                        group_pose = groups_in_shelf[obj[1]][0]
                    except TypeError:
                        pass  # for now, later new group
                    place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(),
                                                      "left",
                                                      world, 0.1, object_desig=obj[0])
                    nearest_pose_to_group = get_closest_pose(group_pose, place_poses)
                else:
                    place_poses = []
                    for link in links_from_shelf:
                        place_poses.append(find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(),
                                                               "left",
                                                               world, 0.1, object_desig=obj[0]))
                    # in this case its the biggest group since why not? i assume  most poses are then free
                    longest_group = max(place_poses, key=len)
                    nearest_pose_to_group = longest_group[0]
                print(nearest_pose_to_group)
                if nearest_pose_to_group:
                    world.current_bullet_world.add_vis_axis(nearest_pose_to_group)
                    talk.pub_now("placing to group:" + obj[1])
                    utils.placerino(obj[0], "front", "left", talk, nearest_pose_to_group, kitchen_desig.names)
                    NavigateAction([table_pose]).resolve().perform()
                    # move.query_pose_nav(rotated_shelf_pose)
                    # move.query_pose_nav(table_pose)
                else:
                    talk.pub_now("HELP")
                    break

        print("done")


demo(1)
