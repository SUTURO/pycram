import time
from std_msgs.msg import String
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import HumanDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher

response = [None, None, None]
callback = False
timeout = 10


class NLP_Functions:
    """
    Class that stores important nlp functions for receptionist
    """

    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        self.response = ["", ""]
        self.callback = False
        self.image_switch_publisher = ImageSwitchPublisher()

    def data_cb(self, data):
        """
        function to receive data from nlp via /nlp_out topic
        """
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.response = data.data.split(",")
        for ele in self.response:
            ele.strip()
        self.response.append("None")
        rospy.loginfo(self.response)
        self.callback = True

    def welcome_guest(self, guest: HumanDescription):
        """
        talking sequence to get name and favorite drink of guest
        :param guest: variable to store new information about human
        """

        TalkingMotion("Welcome, please step in front of me and come close").perform()

        # look for human and position higher
        DetectAction(technique='human').resolve().perform()
        rospy.sleep(1)
        MoveJointsMotion(["torso_lift_joint"], [0.2]).perform()

        # look at guest and introduce
        HeadFollowMotion(state="start").perform()
        rospy.sleep(2.3)
        HeadFollowMotion(state="start").perform()

        TalkingMotion("Hello, i am Toya and my favorite drink is oil.").perform()
        rospy.sleep(2.5)
        TalkingMotion("What is your name and favorite drink?").perform()
        rospy.sleep(2.5)
        TalkingMotion("please answer me when my display changes").perform()
        rospy.sleep(2.5)

        # signal to start listening
        rospy.loginfo("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.2)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        # wait for nlp answer
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                rospy.logwarn("guest needs to repeat")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        self.callback = False

        # check response -> was everything understood with right intent
        if self.response[0] == "<GUEST>":
            # success a name and intent was understood
            if self.response[1].strip() != "None" and self.response[2].strip() != "None":
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
                    # ask for drink again
                    drink = True
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
                self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

                start_time = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time() - start_time) == timeout:
                        rospy.logwarn("guest needs to repeat")
                        self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                self.callback = False

                if self.response[0] == "<GUEST>":
                    # success a name and intent was understood
                    if self.response[1].strip() != "None" and self.response[2].strip() != "None":
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
        rospy.sleep(1)
        TalkingMotion("Nice to meet you").perform()
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
            rospy.sleep(2.5)
            self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

            # wait for response
            start_time = time.time()
            while not self.callback:
                # signal repeat to human
                if time.time() - start_time == timeout:
                    rospy.logwarn("guest needs to repeat")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

            self.image_switch_publisher.pub_now(ImageEnum.HI.value)
            self.callback = False

            if self.response[0] == "<GUEST>" and self.response[1].strip() != "None":
                return self.response[1]

        trys += 1

    def drink_repeat(self):
        """
        HRI-function to ask for drink again.
        """

        self.callback = False
        trys = 0

        while trys < 2:
            TalkingMotion("i am sorry, please repeat your drink loud and clear").perform()
            rospy.sleep(3.5)
            TalkingMotion("please use the sentence my favorite drink is").perform()
            rospy.sleep(3)

            self.nlp_pub.publish("start")
            rospy.sleep(2.5)
            self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

            # wait for response
            start_time = time.time()
            while not self.callback:
                if time.time() - start_time == timeout:
                    rospy.logwarn("guest needs to repeat")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

            self.image_switch_publisher.pub_now(ImageEnum.HI.value)
            self.callback = False

            if self.response[0] == "<GUEST>" and self.response[2].strip() != "None":
                trys += 1
                return self.response[2]

            trys += 1

        return "water"
