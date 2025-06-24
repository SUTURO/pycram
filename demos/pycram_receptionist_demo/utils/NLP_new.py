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
timeout = 12
timeout2 = 17


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

        TalkingMotion("waiting for guests").perform()

        # look for human and position higher
        DetectAction(technique='human_receptionist').resolve().perform()
        TalkingMotion("please come closer").perform()
        rospy.sleep(2)
        TalkingMotion("thank you").perform()

        MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()

        # look at guest and introduce
        HeadFollowMotion(state="start").perform()
        rospy.sleep(1)

        TalkingMotion("Hello, i am Toya").perform()
        rospy.sleep(1.1)
        TalkingMotion("What is your name?").perform()
        rospy.sleep(1.1)
        # TalkingMotion("please use the sentence my name is").perform()
        # rospy.sleep(1.5)
        TalkingMotion("answer me when my display changes").perform()
        rospy.sleep(2.3)

        # signal to start listening
        rospy.loginfo("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.1)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
        # self.image_switch_publisher.pub_now(ImageEnum.TALKING_DUMMIES.value)

        # wait for nlp answer
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                start_time = time.time()
                rospy.logwarn("guest needs to repeat")
                print("listen again")
                self.nlp_pub.publish("start listening")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)


            # if int(time.time() - start_time) == timeout2:
            #     print("listen again")
            #     self.nlp_pub.publish("start listening")
            #     start_time = time.time()

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
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        TalkingMotion(f"Nice to meet you {guest.name}").perform()
        #TalkingMotion(f"Nice to meet you Rania").perform()

        return guest

    def name_repeat(self):
        """
        HRI-function to ask for name again once.
        """

        self.callback = False
        trys = 0

        while trys < 2:
            self.image_switch_publisher.pub_now(ImageEnum.CLOCK.value)
            TalkingMotion("i am sorry, please repeat your name").perform()
            self.image_switch_publisher.pub_now(ImageEnum.CLOCK.value)
            rospy.sleep(2)
            TalkingMotion("use the sentence my name is").perform()
            rospy.sleep(1.2)

            self.nlp_pub.publish("start")
            rospy.sleep(2.5)
            self.image_switch_publisher.pub_now(ImageEnum.TALKING_DUMMIES.value)

            # wait for response
            start_time = time.time()
            while not self.callback:
                # signal repeat to human
                if time.time() - start_time == timeout:
                    rospy.logwarn("guest needs to repeat")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                if int(time.time() - start_time) == timeout2:
                    print("listen again")
                    self.nlp_pub.publish("start listening")
                    start_time = time.time()

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
        # self.image_switch_publisher.pub_now(ImageEnum.TALKING_DUMMIES.value)

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
        TalkingMotion("answer me when my display changes").perform()

        trys = 0
        # wait for nlp answer
        start_time = time.time()
        second = False
        while trys < 2:
            # signal to start listening
            if not second:
                rospy.loginfo("nlp start")
                self.nlp_pub.publish("start listening")
                rospy.sleep(2.1)
                self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
                # self.image_switch_publisher.pub_now(ImageEnum.TALKING_DUMMIES.value)

            while not self.callback and trys < 2:
                print(time.time() - start_time)
                rospy.sleep(1)

                if int(time.time() - start_time) == timeout:
                    rospy.logwarn("guest needs to repeat")
                    print("listen again")
                    self.nlp_pub.publish("start listening")
                    rospy.sleep(1.3)
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                    trys += 1
                # if int(time.time() - start_time) == timeout2:
                #     trys += 1
                #     print("listen again")
                #     self.nlp_pub.publish("start listening")
                #     start_time = time.time()

            self.callback = False
            if self.response[0] == "<INTERESTS>" and self.response[1].strip() != "None":
                rospy.loginfo("interest understood")
                return eval(self.response[1])
            else:
                trys += 1
                print("understood wrong")
                self.nlp_pub.publish("start listening")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                start_time = time.time()
                second = True

        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        return None

    def get_fav_drink(self, guest: HumanDescription):
        """
        sequence in which robot asks person for favorite drink and stores it
        :param guest: variable that stores favorite drink
        """
        TalkingMotion("What is your favorite drink?").perform()
        rospy.sleep(2)
        TalkingMotion("please answer me when my display changes").perform()
        rospy.sleep(2.2)

        # signal to start listening
        self.nlp_pub.publish("start listening")
        rospy.loginfo("nlp start")
        rospy.sleep(2.1)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
        #self.image_switch_publisher.pub_now(ImageEnum.TALKING_DUMMIES.value)

        # wait for nlp answer
        start_time = time.time()
        trys = 0
        while not self.callback and trys < 2:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                rospy.logwarn("guest needs to repeat")
                print("listen again")
                self.nlp_pub.publish("start listening")
                start_time = time.time()
                rospy.sleep(1)
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                trys += 1
            # if int(time.time() - start_time) == timeout2:
            #     print("listen again")
            #     self.nlp_pub.publish("start listening")
            #     start_time = time.time()

        self.callback = False

        # check response -> was everything understood with right intent
        if self.response[0] == "<GUEST>" and self.response[2].strip() != "None":
            guest.set_drink(self.response[2])
        else:
            guest.set_drink(self.drink_repeat())
        if guest.fav_drink:
            self.image_switch_publisher.pub_now(ImageEnum.HI.value)
            TalkingMotion(f"your favorite drink is {guest.fav_drink}").perform()


    def drink_repeat(self):
        """
        HRI-function to ask for drink again.
        """

        self.callback = False
        trys = 0

        while trys < 1:
            TalkingMotion("i am sorry, please repeat your drink loud and clear").perform()
            self.image_switch_publisher.pub_now(ImageEnum.CLOCK.value)
            rospy.sleep(3.5)
            TalkingMotion("please use the sentence my favorite drink is").perform()
            rospy.sleep(2.5)

            self.nlp_pub.publish("start")
            rospy.sleep(2)
            self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

            # wait for response
            start_time = time.time()
            while not self.callback and trys < 1:
                if time.time() - start_time == timeout:
                    rospy.logwarn("guest needs to repeat")
                    self.nlp_pub.publish("start listening")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

                if int(time.time() - start_time) == timeout2:
                    trys += 1
                    start_time = time.time()

                    # print("listen again")
                    # self.nlp_pub.publish("start listening")
                    # start_time = time.time()

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
            if hobby_list[0] == 'playing':
                hobby_list.remove('playing')
            guest.add_interests(hobby_list[0])
        if hobby_list:
            for indx in range(len(hobby_list)):
                hobby_list[indx] = hobby_list[indx].lower()


        # answer specifically
        toya_text = self.res_loader.predict_response(hobby_list)
        print(guest.interests)
        if guest.interests:
            TalkingMotion(f"you like {guest.interests[0]}").perform()
        TalkingMotion(toya_text).perform()
