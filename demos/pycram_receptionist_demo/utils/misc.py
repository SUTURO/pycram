import time

import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import *
from pycram.designators.object_designator import HumanDescription

from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter

image_switch_publisher = ImageSwitchPublisher()



def datamisc_cb(data):
    global response
    global callback


    image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    for ele in response:
        ele.strip()
    response.append("None")
    print(response)
    callback = True


timeout = 10



def get_attributes(guest: HumanDescription):
    """
    storing attributes and face of person in front of robot
    :param guest: variable to store information in
    """
    # remember face
    keys = DetectAction(technique='human', state='face').resolve().perform()
    keys1 = keys[1]
    new_id = keys["keys"][0]
    guest.set_id(new_id)

    # get clothes and gender
    attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
    print(attr_list)
    #guest.set_attributes(attr_list)
    rospy.loginfo(attr_list)


def name_repeat():
    """
    HRI-function to ask for name again once.
    """

    # receive data from nlp via topic
    rospy.Subscriber("nlp_out", String, datamisc_cb)
    pub_nlp2 = rospy.Publisher('/startListener', String, queue_size=16)

    global callback
    global response
    global wait_bool
    callback = False
    trys = 0

    while trys < 2:
        TalkingMotion("i am sorry, please repeat your name").perform()
        rospy.sleep(1.2)
        pub_nlp2.publish("start")

        # sound/picture
        rospy.sleep(3)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not callback:
            # signal repeat to human
            if time.time() - start_time == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        image_switch_publisher.pub_now(ImageEnum.HI.value)
        callback = False

        if response[0] == "<GUEST>" and response[1].strip() != "None":
            return response[1]

    trys += 1


def drink_repeat():
    """
    HRI-function to ask for drink again once.
    """

    # receive data from nlp via topic
    rospy.Subscriber("nlp_out", String, datamisc_cb)
    pub_nlp3 = rospy.Publisher('/startListener', String, queue_size=16)
    global callback
    # global response
    global wait_bool
    callback = False
    trys = 0

    while trys < 2:
        TalkingMotion("i am sorry, please repeat your drink loud and clear").perform()
        rospy.sleep(3.5)
        TalkingMotion("please use the sentence my favorite drink is").perform()
        rospy.sleep(3.1)
        pub_nlp3.publish("start")

        # sound/picture
        rospy.sleep(3)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not callback:
            # signal repeat to human
            if time.time() - start_time == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        image_switch_publisher.pub_now(ImageEnum.HI.value)
        callback = False

        if response[0] == "<GUEST>" and response[2].strip() != "None":
            trys += 1
            return response[2]

        trys += 1

    return "water"
