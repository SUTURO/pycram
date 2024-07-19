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

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
move = PoseNavigator()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalmug"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

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


def navigate_and_detect():
    """
    Navigates to the couch table and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    global sorted_obj_len
    text_to_speech_publisher.pub_now("Navigating")

    move.pub_now(goal_pose)
    MoveTorsoAction([0.25]).resolve().perform()
    MoveJointsMotion(["arm_roll_joint"], [1.5]).resolve().perform()
    LookAtAction(targets=[look_at_pose]).resolve().perform()

    text_to_speech_publisher.pub_now("Perceiving")
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


with real_robot:
    rospy.loginfo("Starting demo")
    perceiving_y_pos = 4.76
    look_at_pose = Pose([7.8, perceiving_y_pos, 0.25])
    goal_pose = Pose([8.8, perceiving_y_pos, 0], [0, 0, 1, 0])
    MoveGripperMotion("open", "left").resolve().perform()

    while True:
        object_desig = navigate_and_detect()
        sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)

        grasps = "front"
        if not sorted_obj:
            text_to_speech_publisher.pub_now("Nothing found")
            continue
        else:
            if sorted_obj[0].type in CUTLERY or sorted_obj[0].type == "Metalbowl":
                grasps = "top"
            PickUpAction(sorted_obj[0], ["left"], [grasps]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            text_to_speech_publisher.pub_now("Dropping object")
            time.sleep(1)
            MoveGripperMotion("open", "left").resolve().perform()

