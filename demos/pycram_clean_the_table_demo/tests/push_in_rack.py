
from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import StartSignalWaiter
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher


# Create an instance of the StartSignalWaiter


# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Fork", "Spoon", "Metalmug"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# ForceTorqueSensor for recognizing push on the hand
fts = ForceTorqueSensor(robot_name='hsrb')

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_clean_v1.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()

move = PoseNavigator()

with real_robot:
    rospy.loginfo("Starting demo")
    MoveGripperMotion("close","left").resolve().perform()

    move.pub_now(Pose([8.9, 4.72, 0], [0, 0, 0, 1]))
    config_for_pushing_rack = {'arm_flex_joint': -1.6, 'arm_lift_joint': 0.05, 'arm_roll_joint': 0,
                          'wrist_flex_joint': -1.4, 'wrist_roll_joint': -0.4}

    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config_for_pushing_rack)
    text_to_speech_publisher.pub_now("Pushing")
    MoveJointsMotion(["wrist_flex_joint"], [0.0]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()