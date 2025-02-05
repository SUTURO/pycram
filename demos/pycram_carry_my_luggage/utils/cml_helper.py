import pycram.external_interfaces.giskard as giskardpy
import rospy
from robokudo_msgs.msg import QueryAction, QueryGoal
from roslibpy import actionlib

from demos.pycram_carry_my_luggage.utils.cml_human import Human
from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import NavigateAction, DetectAction
from pycram.designators.motion_designator import TalkingMotion, MoveJointsMotion
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.language import Code, Monitor
from pycram.ros.action_lib import create_action_client
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.utilities.robocup_utils import ImageSwitchPublisher

img = ImageSwitchPublisher()
fts = ForceTorqueSensor(robot_name='hsrb')
rkclient = create_action_client('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
rkclient.wait_for_server()
rospy.loginfo("You can start your demo now")









def giskard_drive(human):
    DetectAction(technique='human').resolve().perform()
    try:
        plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func())
        plan.perform()
    except SensorMonitoringCondition:
        print("CHANGE BACK")
        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        TalkingMotion("Boop beep").perform()
