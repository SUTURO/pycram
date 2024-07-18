import time
from enum import Enum

import geometry_msgs.msg
import rospy.core
import tf

from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import StartSignalWaiter
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
move = PoseNavigator()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the couch table
table_pose = 4.84

# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"

# Intermediate positions for a safer navigation
move_to_the_middle_table_pose = [2.2, 1.98, 0]
move_to_the_middle_dishwasher_pose = [2.2, -0.1, 0]

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")

dishwasher_main_name = "sink_area_dish_washer_main"


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


def calculate_placing_pos(x_pos, y_pos, z_pos):
    lt = LocalTransformer()

    link = apartment.get_link_tf_frame(dishwasher_main_name)

    world.current_bullet_world.add_vis_axis(apartment.get_link_pose(dishwasher_main_name))
    dishwasher = Pose([x_pos, y_pos, z_pos], [0, 0, 0, 1])
    newp = lt.transform_pose(dishwasher, link)  # link statt map wenn 1) verwendet. map wenn 2) verwendet
    print(newp)
    world.current_bullet_world.add_vis_axis(newp)
    return newp.pose


with real_robot:
    rospy.loginfo("Starting demo")
    # TalkingMotion("Starting demo").resolve().perform()
    pos = get_placing_pos("Metalmug")
    print(f"Metalmug: {pos}")
    if pos.position.y >= get_placing_pos("check").position.y:
        print("left")
    else:
        print("right")
    pos = get_placing_pos("Metalplate")
    print(f"Metalplate: {pos}")
    if pos.position.y >= get_placing_pos("check").position.y:
        print("left")
    else:
        print("right")
    pos = get_placing_pos("Metalbowl")
    print(f"Metalbowl: {pos}")
    if pos.position.y >= get_placing_pos("check").position.y:
        print("left")
    else:
        print("right")
    pos = get_placing_pos("Cutlery")
    print(f"Cutlery: {pos}")
    if pos.position.y >= get_placing_pos("check").position.y:
        print("left")
    else:
        print("right")

    text_to_speech_publisher.pub_now("living room")
    goal_pose = Pose([5.8, 0.2, 0], [0, 0, 0, 1])
    move.pub_now(goal_pose)
    time.sleep(1)

    text_to_speech_publisher.pub_now("middle living room")
    goal_pose = Pose([5.8, 0.2, 0], [0, 0, 0, 1])
    move.pub_now(goal_pose)
    time.sleep(1)

    text_to_speech_publisher.pub_now("kitchen")
    goal_pose = Pose([9.2, 3.08, 0], [0, 0, 0.7, 0.7])
    move.pub_now(goal_pose)
    time.sleep(1)

    text_to_speech_publisher.pub_now("Dishwasher")
    goal_pose = Pose([9.4, 4.74, 0], [0, 0, 0, 1])
    move.pub_now(goal_pose)
    time.sleep(4)

    text_to_speech_publisher.pub_now("Table")
    goal_pose = Pose([8.5, 4.68, 0], [0, 0, 1, 0])
    move.pub_now(goal_pose)
    time.sleep(2)

    text_to_speech_publisher.pub_now("Dishwasher turn around")
    goal_pose = Pose([9.1, 4.8, 0], [0, 0, 0, 1])
    move.pub_now(goal_pose)
    time.sleep(4)

    text_to_speech_publisher.pub_now("Dishwasher left")
    goal_pose = Pose([9.8, 4.3, 0], [0, 0, -0.7, 0.7])
    move.pub_now(goal_pose)
    time.sleep(4)

    text_to_speech_publisher.pub_now("Dishwasher right")
    goal_pose = Pose([9.8, 5.1, 0], [0, 0, 0.7, 0.7])
    move.pub_now(goal_pose)
    time.sleep(4)

    text_to_speech_publisher.pub_now("Done")