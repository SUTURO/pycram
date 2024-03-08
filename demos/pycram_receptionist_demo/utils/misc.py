import rospy
from pycram.designators.action_designator import NavigateAction
from pycram.designators.motion_designator import TalkingMotion
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *
from std_msgs.msg import String
from demos.pycram_receptionist_demo.deprecated import talk_actions
from deprecated import deprecated

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def talk_request(data: list):
    """
    callback function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: String "name drink"
    """

    rospy.loginfo("in callback success")
    toyas_text = "Hey " + data[0] + " your favorite drink is " + data[
        1]
    TalkingMotion(toyas_text).resolve().perform()
    rospy.sleep(1)
    TalkingMotion("nice to meet you").resolve().perform()


def talk_request_nlp(data: str):
    """
    callback function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: String "name drink"
    """

    rospy.loginfo("in callback success")
    data = data.split(",")
    toyas_text = "Hey " + data[0] + " your favorite drink is " + data[
        1]
    TalkingMotion(toyas_text).resolve().perform()
    rospy.sleep(1)
    TalkingMotion("nice to meet you").resolve().perform()


def talk_error(data):
    """
    callback function if no name/drink was heard
    """

    error_msgs = "i could not hear you, please repeat"
    TalkingMotion(error_msgs).resolve().perform()
    rospy.sleep(2)
    pub_nlp.publish("start listening")


def introduce(name1, drink1, host_name, host_drink):
    """
    Text for robot to introduce two people to each other
    """
    first = "Hey " + str(host_name) + " This is " + str(name1) + " and the favorite drink of your guest is " + str(drink1)

    second = str(name1) + " This is " + str(host_name) + " his favorite drink is " + str(host_drink)
    TalkingMotion(first).resolve().perform()
    rospy.sleep(3)
    TalkingMotion(second).resolve().perform()


