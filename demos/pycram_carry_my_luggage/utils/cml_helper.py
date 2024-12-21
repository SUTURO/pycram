import rospy
from robokudo_msgs.msg import QueryAction, QueryGoal
from roslibpy import actionlib

from demos.pycram_carry_my_luggage.utils.cml_human import Human
from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.language import Code, Monitor
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.utilities.robocup_utils import ImageSwitchPublisher

img = ImageSwitchPublisher()
fts = ForceTorqueSensor(robot_name='hsrb')
rkclient = actionlib.SimpleActionClient('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
rkclient.wait_for_server()
rospy.loginfo("You can start your demo now")


def monitor_func_human(human: Human):
    """
    monitors gripper state and if human is still in field of vision
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor")
        return SensorMonitoringCondition
    if not human.human_pose.get_value():
        rospy.logerr("lost human")
        return HumanNotFoundCondition
    return False


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def lost_human(human: Human):
    """
    function to handle case the robot loses vision of the human it is following
    :param human: variable of human that is lost
    """
    no_human = True
    while no_human:
        goal_msg = QueryGoal()
        if human.human_pose.get_value():
            no_human = False
        else:
            rospy.sleep(5)
            TalkingMotion("I lost you, please step in front of me").perform()
            rkclient.send_goal(goal_msg)


def drive_back_move_base(start_pose):
    """
    navigate with move base to the start point of the challenge
    :param start_pose: pose the robot navigates to
    """
    rospy.loginfo("driving back")
    TalkingMotion("Driving Back.").perform()
    img.pub_now(ImageEnum.DRIVINGBACK.value)
    NavigateAction(start_pose).resolve().perform()


def demo_start(human: Human):
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """
    try:
        TalkingMotion("Starting Carry my Luggage demo.").perform()
        img.pub_now(ImageEnum.HI.value)  # hi im toya
        TalkingMotion("Push down my Hand, when you are Ready.").perform()
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        img.pub_now(ImageEnum.HI.value)

        TalkingMotion("Looking for a human").perform()
        img.pub_now(ImageEnum.SEARCH.value)
        goal_msg = QueryGoal()
        rkclient.send_goal(goal_msg)
        rospy.loginfo("Waiting for human to be detected")
        no_human = True
        while no_human:

            if human.human_pose.wait_for(timeout=5):
                no_human = False
            else:
                TalkingMotion("Looking for a human, please step in front of me").perform()
                rkclient.send_goal(goal_msg)

        TalkingMotion("Found a Human").perform()
        img.pub_now(ImageEnum.HI.value)
