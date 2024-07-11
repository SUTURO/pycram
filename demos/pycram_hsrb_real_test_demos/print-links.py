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




# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_clean_v1.urdf")
apart_desig = BelieveObject(names=["kitchen"])
robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf")
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
rospy.sleep(2)

print(robot.get_complete_joint_state())