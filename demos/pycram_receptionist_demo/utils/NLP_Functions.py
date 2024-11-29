import time
from typing import Optional

import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import HumanDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher

response = [None, None, None]
callback = False
timeout = 10
image_switch_publisher = ImageSwitchPublisher()





class NLP_Functions:
    """
    Class that stores important nlp functions for receptionist
    """

    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        self.response = ["", ""]
        self.callback = False

    def data_cb(self, data):
        """
        function to receive data from nlp via /nlp_out topic
        """

        image_switch_publisher = ImageSwitchPublisher()

        image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.response = data.data.split(",")
        for ele in self.response:
            ele.strip()
        self.response.append("None")
        print(self.response)
        self.callback = True

    def welcome_guest(self, guest: HumanDescription):
        """
        talking sequence to get name and favorite drink of guest
        and attributes if it is the first guest
        :param num: number of guest
        :param guest: variable to store information in
        """

        TalkingMotion("Welcome, please step in front of me and come close").perform()

        # look for human
        DetectAction(technique='human').resolve().perform()
        rospy.sleep(1)

        # look at guest and introduce
        HeadFollowMotion(state="start").perform()
        # giskardpy.move_head_to_human()
        rospy.sleep(2.3)

        TalkingMotion("Hello, i am Toya and my favorite drink is oil.").perform()
        rospy.sleep(3)

        TalkingMotion("What is your name and favorite drink?").perform()
        rospy.sleep(2.5)
        TalkingMotion("please answer me when my display changes").perform()
        rospy.sleep(2.5)

        # signal to start listening
        print("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)

        # wait for nlp answer
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        self.callback = False

        if self.response[0] == "<GUEST>":
            # success a name and intent was understood
            if self.response[1].strip() != "None" and self.response[2].strip() != "None":
                print("in success")
                # understood both
                guest.set_drink(self.response[2])
                guest.set_name(self.response[1])
            else:
                name = False
                drink = False
                if self.response[1].strip() == "None":
                    # ask for name again once
                    name = True
                    guest.set_drink(self.response[2])

                if self.response[2].strip() == "None":
                    drink = True
                    # ask for drink again
                    guest.set_name(self.response[1])

                if name:
                    guest.set_name(self.name_repeat())

                if drink:
                    guest.set_drink(self.drink_repeat())

        else:
            # two chances to get name and drink
            i = 0
            while i < 2:
                TalkingMotion("please repeat your name and drink loud and clear").perform()
                rospy.sleep(2.1)

                self.nlp_pub.publish("start")
                rospy.sleep(2.5)
                image_switch_publisher.pub_now(ImageEnum.TALK.value)

                start_time = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time() - start_time) == timeout:
                        print("guest needs to repeat")
                        image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                self.callback = False

                if self.response[0] == "<GUEST>":
                    # success a name and intent was understood
                    if self.response[1].strip() != "None" and self.response[2].strip() != "None":
                        print("in success")
                        # understood both
                        guest.set_drink(self.response[2])
                        guest.set_name(self.response[1])
                        break
                    else:
                        name = False
                        drink = False
                        if self.response[1].strip() == "None":
                            # ask for name again once
                            name = True
                            guest.set_drink(self.response[2])

                        if self.response[2].strip() == "None":
                            drink = True
                            # ask for drink again
                            guest.set_name(self.response[1])

                        if name:
                            guest.set_name(self.name_repeat())
                            break

                        if drink:
                            guest.set_drink(self.drink_repeat())
                            break

        HeadFollowMotion(state="stop").perform()
        DetectAction(technique='human', state="stop").resolve().perform()
        return guest

    def name_repeat(self):
        """
        HRI-function to ask for name again once.
        """

        self.callback = False
        trys = 0

        while trys < 2:
            TalkingMotion("i am sorry, please repeat your name").perform()
            rospy.sleep(1.2)
            self.nlp_pub.publish("start")

            # sound/picture
            rospy.sleep(2.5)
            image_switch_publisher.pub_now(ImageEnum.TALK.value)

            start_time = time.time()
            while not self.callback:
                # signal repeat to human
                if time.time() - start_time == timeout:
                    print("guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

            image_switch_publisher.pub_now(ImageEnum.HI.value)
            self.callback = False

            if self.response[0] == "<GUEST>" and self.response[1].strip() != "None":
                return self.response[1]

        trys += 1

    def drink_repeat(self):
        """
        HRI-function to ask for drink again once.
        """

        self.callback = False
        trys = 0

        while trys < 2:
            TalkingMotion("i am sorry, please repeat your drink loud and clear").perform()
            rospy.sleep(3.5)
            TalkingMotion("please use the sentence my favorite drink is").perform()
            rospy.sleep(3.1)
            self.nlp_pub.publish("start")

            # sound/picture
            rospy.sleep(3)
            image_switch_publisher.pub_now(ImageEnum.TALK.value)

            start_time = time.time()
            while not self.callback:
                # signal repeat to human
                if time.time() - start_time == timeout:
                    print("guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

            image_switch_publisher.pub_now(ImageEnum.HI.value)
            self.callback = False

            if self.response[0] == "<GUEST>" and self.response[2].strip() != "None":
                trys += 1
                return self.response[2]

            trys += 1

        return "water"
