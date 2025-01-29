import time
from typing import List

import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import DetectAction
from pycram.designators.motion_designator import TalkingMotion, HeadFollowMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher
import re

response = [None, None]
confirmation = [None]
callback = False
timeout = 10

image_switch_publisher = ImageSwitchPublisher()
options = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
           '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10,}


class nlp_restaurant:
    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        self.sub_nlp_confirm = rospy.Subscriber("nlp_out", String, self.data_cb_confirmation)
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

    def data_cb_confirmation(self, data):
        image_switch_publisher = ImageSwitchPublisher()
        print(data)
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.confirmation = data.data.split(",")
        print(self.confirmation)
        self.callback = True
    def order_ready(self):
        """
        Confirmation from the bartender that order is ready.
        :param data: The response
        :return: Boolean
        """
        TalkingMotion("Please confirm that the order is ready.").perform()
        rospy.sleep(2)

        rospy.loginfo("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if int(time.time()) - start_time == timeout:
                rospy.loginfo("Please repeat")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
        self.callback = False
        print(self.response)
        if self.response[0] == "<CONFIRM>":
            return True
        elif self.response[0] == "<DENY>":
            rospy.sleep(4)




    def split_response(self, data):
        """
        Removes all symbols that are not letters.
        :param: data: The response as a split list
        :return: A clean list
        """
        new_tmp = [n.strip() for n in data]
        real_msg = [re.sub('\W+', '', m) for m in new_tmp]
        return real_msg

    def save_order(self, data):
        """
        Creates a list of tuples based on the data input of NLP.
        :param: data: The list of received order
        :return: A list of tuples
        """
        tuple_order = [(x, options[y]) for x, y in zip(data, data[1:]) if y in options]
        return tuple_order

    def confirm_order(self, customer: CustomerDescription, order: [(str, int)]):
        """
        Method to confirm an order with the current customer. If Toya did not understand the order
        correctly, the order process will be repeated.

        :param: order: The order of the current customer
        """
        print("Confirm oder", order)
        if len(order) == 1:
            TalkingMotion(f"Do you want to order {order[0][1]} {order[0][0]}?").perform()
            rospy.sleep(2.5)
            print("nlp start")
            self.nlp_pub.publish("start listening")
            rospy.sleep(2.3)

            start_time = time.time()
            while not self.callback:
                rospy.sleep(1)
                if int(time.time()) - start_time == timeout:
                    rospy.logwarn("Guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            self.callback = False
            if self.response[0] == "<CONFIRM>":
                return True
            else:
                self.get_order(customer=customer)

        # else:
        #     for n in order:
        #         TalkingMotion(f"Do you want to order {n[1]} {n[0]}?").perform()
        #
           # start_time = time.time()
           # while not self.callback:
           #     rospy.sleep(1)

            #    if int(time.time()) - start_time == timeout:
             #       rospy.logwarn("Guest needs to repeat")
              #      image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

            #self.callback = False
            #print(self.response)
            #if self.response[0] == "<Hobbies>":
             #   print(self.response)
            #print(self.confirmation)

    def get_order(self, customer: CustomerDescription):
        """
        Method to order food if Toya successfully arrived at a customer.
        :param: customer: The customer that wants to oder something
        """
        global order
        TalkingMotion("Welcome, what can I get for you? Please come close to me and order when my display changes").perform()
        rospy.sleep(4)


        print("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)

            if int(time.time() - start_time) == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                rospy.sleep(2)

        self.callback = False

        if self.response[0] == "<ORDER>":
            tmp = self.split_response(self.response)
            order = self.save_order(tmp)
            if order is not None:
                customer.set_order(order)

        else:
            tries = 0
            while tries < 2:
                rospy.sleep(2.3)

                self.nlp_pub.publish("start")
                self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

                start_time_rep = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time() - start_time) == timeout:
                        rospy.logwarn("guest needs to repeat")
                        self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                        rospy.sleep(2)

                self.callback = False
                if self.response[0] == "<ORDER>":
                    tmp_rep = self.split_response(self.response)
                    order_rep = self.save_order(tmp_rep)
                    if order_rep is not None:
                        customer.set_order(order_rep)
                        break
                    else:
                        print(tries)
                        tries += 1

