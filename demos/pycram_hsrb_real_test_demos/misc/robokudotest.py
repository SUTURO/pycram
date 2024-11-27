import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped
from robokudo_msgs.msg import QueryAction, QueryGoal

from pycram.designators.action_designator import fts, ParkArmsAction, DetectAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.plan_failures import SensorMonitoringCondition
from pycram.process_module import real_robot
from utils.startup import startup
import pycram.external_interfaces.giskard_new as giskardpy

# Initialize the necessary components
world, v, text_to_speech_publisher, image_switch_publisher, move = startup()
giskardpy.init_giskard_interface()
client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
client.wait_for_server()
rospy.loginfo("You can start your demo now")

with real_robot:

    DetectAction(technique='all').resolve().perform()