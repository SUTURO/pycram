import json
import time

import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import DetectAction
from pycram.designators.motion_designator import TalkingMotion, HeadFollowMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher

response = [None, None]
callback = False
dict_response = {}
timeout = 10
global tmp_order
image_switch_publisher = ImageSwitchPublisher()


class NLPRestaurant:
    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        self.response = ["", ""]
        self.callback = False
        self.direct_data = String("")

    def stnd_msg_to_dict(self, msg: String):
        tmp_dump = json.loads(msg.data)
        print(tmp_dump)
        return tmp_dump

    def data_cb(self, data):
        """
        function to receive data from nlp via /nlp_out topic
        """
        print(type(data))
        image_switch_publisher = ImageSwitchPublisher()
        self.direct_data = data
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.response = data.data.split(",")
        for ele in self.response:
            ele.strip()
        self.response.append("None")
        print(self.response)
        self.callback = True

    def order_repeat(self, customer: CustomerDescription):
        global tmp_order
        self.callback = False
        tries= 0

        while tries <= 2:
            TalkingMotion("I am sorry, please repeat your order").perform()
            rospy.sleep(1.2)
            self.nlp_pub.publish("start")

            start_time = time.time()
            while not self.callback:
                if time.time() - start_time == timeout:
                    print("customer needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            self.callback = False
            if self.response != ["None"]:
                tmp_order = self.stnd_msg_to_dict(self.direct_data)
            if self.response[0] == "<ORDER>" and tmp_order is not None:
                return tmp_order
        tries += 1



    def get_order(self, customer: CustomerDescription):
        """
        Method to order food if Toya successfully arrived at a customer.
        :param: customer: The customer that wants to oder something
        """
        TalkingMotion("Welcome, what can I get for you? Please come close to me").perform()

        DetectAction(technique='human').resolve().perform()
        rospy.sleep(1)

        HeadFollowMotion(state='start').perform()

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

        if self.response[0] == "<ORDER>":
            tmp_order = self.stnd_msg_to_dict(self.direct_data)
            print(tmp_order)
            if len(tmp_order) != 0:
                for ord in tmp_order:
                    if ord[0] is not None and ord[1] is not None:
                        customer.set_order(ord[0], ord[1])
                        print(customer.order)
                    else:
                        self.order_repeat(customer)
            else:
                order = False
                amount = False
                if self.response[1].strip() != "None":
                    order = True
                    customer.set_order(self.response[1].strip(), 1)


