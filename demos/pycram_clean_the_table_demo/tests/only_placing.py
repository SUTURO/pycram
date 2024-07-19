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

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
move = PoseNavigator()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

move_to_the_middle_dishwasher_pose = Pose([8.9, 4.72, 0], [0, 0, 0, 1])
# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"
dishwasher_main_name = "sink_area_dish_washer_main"
placing_location_name_left = "dishwasher_left"
placing_location_name_right = "dishwasher_right"
dishwasher_pose = "dishwasher_front"
pickup_location_name = "dinner_table"
# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_clean_v1.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()


def get_placing_pos(obj):
    lt = LocalTransformer()
    dishwasher_main_name = "sink_area_dish_washer_main"
    link = apartment.get_link_tf_frame(dishwasher_main_name)

    world.current_bullet_world.add_vis_axis(apartment.get_link_pose(dishwasher_main_name))
    if obj == "Cutlery":
        # z = 0.48
        dishwasher = Pose([0.7225437770469778, 0.035078306407233306, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Metalbowl":
        dishwasher = Pose([0.5081242147912377, 0.15559490825966282, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Metalmug":
        dishwasher = Pose([0.5057047928413176, 0.10565347859021657, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Metalplate":
        # z = 0.55
        dishwasher = Pose([0.6229715158422948, -0.16033245276069596, 0.11000000238418584],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Dishwashertab":
        # todo: Werte ändern
        dishwasher = Pose([0.5988132707204343, -0.03902136838407344, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])

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
    if location_name == placing_location_name_left:
        print("left")
        goal_pose = Pose([9.8, 4.3, 0], [0, 0, -0.7, 0.7])

        move.pub_now(move_to_the_middle_dishwasher_pose)
        while not check_position():
            move.pub_now(goal_pose)

    elif location_name == dishwasher_pose:
        goal_pose = Pose([9.18, 4.72, 0], [0, 0, 0, 1])

        move.pub_now(move_to_the_middle_dishwasher_pose)
        while not check_position():
            move.pub_now(goal_pose)
    else:
        rospy.logerr(f"Failure. Y-Value must be set for the navigateAction to the {pickup_location_name}")


with real_robot:
    rospy.loginfo("Starting demo")
    text_to_speech_publisher.pub_now("Starting demo")

    obj = "Metalmug"
   # obj = "Metalbowl"
   # obj = "Metalplate"
   # obj = "Fork"

    grasp = "front"

    # if obj == "Metalplate" or obj == "Metalbowl":
    #     MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()

    placing_pose = get_placing_pos(obj)
    # todo: silverware tray must be on the right side of the dishwasher
    if obj in ["Metalbowl", "Metalmug"]:
        navigate_to(placing_location_name_right)
    else:
        navigate_to(dishwasher_pose)

    MoveGripperMotion("open", "left").resolve().perform()
    time.sleep(2)
    text_to_speech_publisher.pub_now("Grasping")
    MoveGripperMotion("close", "left").resolve().perform()

    PlaceGivenObjAction([obj], ["left"], [placing_pose], [grasp]).resolve().perform()
    image_switch_publisher.pub_now(0)
    # For the safety in cases where the HSR is not placing, better drop the object to not colide with the kitchen drawer when moving to parkArms arm config
    MoveGripperMotion("open", "left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

