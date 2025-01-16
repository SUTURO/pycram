import time

import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import DetectAction
from pycram.designators.motion_designator import TalkingMotion, HeadFollowMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher

response = [None, None, None]
callback = False
timeout = 10

image_switch_publisher = ImageSwitchPublisher()


class nlp_restaurant:
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

        image_switch_publisher = ImageSwitchPublisher()
        print(data)
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.response = data.data.split(",")
        for ele in self.response:
            ele.strip()
        self.response.append("None")
        print(self.response)
        self.callback = True

    def repeat_order(self, customer: CustomerDescription):
        TalkingMotion("Do you want to order the following items?").perform()
        TalkingMotion(f"{customer.order[1]}{customer.order[0]} ").perform()

    def get_order(self, customer: CustomerDescription):
        """
        Method to order food if Toya successfully arrived at a customer.
        :param: customer: The customer that wants to oder something
        """
        TalkingMotion("Welcome, what can I get for you? Please come close to me").perform()

        # DetectAction(technique='human').resolve().perform()
        rospy.sleep(1)

        # HeadFollowMotion(state='start').perform()

        print("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        self.callback = False
        print(type(self.response))

        if self.response[0] == "<ORDER>":
            for n in self.response:
                print(n)
                if n != "<ORDER>" or n != "None":
                    print(n.split("'")[1])

            if self.response[1].strip() != "None" and self.response[2].strip() is not None:
                customer.set_order(self.response[1], self.response[2])
            else:
                order = False
                amount = False
                if self.response[1].strip() != "None":
                    order = True
                    customer.set_order(self.response[1].strip(), 1)
