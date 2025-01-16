import time
import rospy
from std_msgs.msg import String

from demos.pycram_receptionist_demo.utils.ResponseLoader import ResponseLoader
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import HumanDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher

response = [None, None, None]
callback = False
timeout = 10


class NLP_Helper:
    """
    Class that stores important nlp functions for receptionist
    """

    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        rospy.sleep(2)
        self.res_loader = ResponseLoader("a.json")
        self.res_loader.load_data()
        self.response = ["", ""]
        self.callback = False
        self.image_switch_publisher = ImageSwitchPublisher()

    def data_cb(self, data):
        """
        function to receive data from nlp via /nlp_out topic
        """
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.response = data.data.split(";")
        print("response: " + str(self.response))
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

        TalkingMotion("Hello, i am Toya").perform()
        rospy.sleep(1.7)
        TalkingMotion("What is your name?").perform()
        rospy.sleep(1.7)
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
            if self.response[1].strip() != "None":
                # understood both
                guest.set_name(self.response[1])
            else:
                guest.set_name(self.name_repeat())

        else:
            # two chances to get name and drink
            guest.set_name(self.name_repeat())

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

    def listen_return_answer(self):
        """
        function that returns whatever the person said
        """

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
        return response

    def listen_return_interest(self):
        """
        function that returns interests of conversational partner
        """
        trys = 0
        while trys < 2:
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
            if self.response[0] == "<INTERESTS>" and self.response[1].strip() != "None":
                rospy.loginfo("interest understood")
                return eval(self.response[1])
            else:
                trys += 1

        return None

    def get_fav_drink(self, guest: HumanDescription):
        """
        sequence in which robot asks person for favorite drink and stores it
        :param guest: variable that stores favorite drink
        """
        TalkingMotion("What is your favorite drink?").perform()
        rospy.sleep(2.5)
        TalkingMotion("please answer me when my display changes").perform()
        rospy.sleep(2.5)

        # signal to start listening
        self.nlp_pub.publish("start listening")
        rospy.loginfo("nlp start")
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
        if self.response[0] == "<GUEST>" and self.response[2].strip() != "None":
            guest.set_drink(self.response[2])
        else:
            guest.set_drink(self.drink_repeat())

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

    def store_and_answer_hobby(self, guest: HumanDescription):
        """
        function to answer to hobby individually
        """

        # get interests
        hobby_list = self.listen_return_interest()

        # store interests
        if hobby_list:
            guest.add_interests(hobby_list)

        # answer specifically
        toya_text = self.res_loader.predict_response(hobby_list)
        print(toya_text)
        TalkingMotion(toya_text).perform()
