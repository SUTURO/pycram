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


def monitor_func():
    der: WrenchStamped() = fts.get_last_value()
    print(abs(der.wrench.force.x))
    if abs(der.wrench.force.x) > 10.30:
        print(abs(der.wrench.force.x))
        print(abs(der.wrench.torque.x))
        return SensorMonitoringCondition
    return False


with real_robot:
    rospy.loginfo("Starting demo")
    MoveGripperMotion("open", "left").resolve().perform()
    text_to_speech_publisher.pub_now("Push down my hand, when I should grasp")
    try:
        plan = Code(lambda: rospy.sleep(1)) * 99999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        rospy.logwarn("Close Gripper")
        MoveGripperMotion(motion="close", gripper="left").resolve().perform()
    print("I'm done")
    print("really I'm done")