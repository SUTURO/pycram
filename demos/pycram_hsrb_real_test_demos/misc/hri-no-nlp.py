import itertools

import rospy
from geometry_msgs.msg import WrenchStamped

import pycram.language
from pycram.designators.action_designator import fts
from pycram.language import Monitor
from pycram.plan_failures import SensorMonitoringCondition
from pycram.ros.force_torque_sensor import ForceTorqueSensor
from utils.startup import startup
from pycram.language import *
from pycram.process_module import simulated_robot

world, v, text_to_speech_publisher, image_switch_publisher, move = startup()
rospy.loginfo("u can start your demo now")


# try:
#     plan = MoveTCPMotion(side_push, self.arm) >> Monitor(monitor_func)
#     plan.perform()
# except (SensorMonitoringCondition):
#     rospy.logwarn("Open Gripper")
#     MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()
# fts = ForceTorqueSensor(robot_name='hsrb')
# pr = True

def monitor_func():
    der: WrenchStamped() = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def printloop():
    print(".")


with simulated_robot:
    try:

        text_to_speech_publisher.publish_text("Starting the demo. You can interrupt me: anytime!")
        plan = Code(lambda: printloop()) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        print("human interrupted")
        text_to_speech_publisher.publish_text("follow up code now")
