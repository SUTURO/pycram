import json
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
global repeat
repeat = False
image_switch_publisher = ImageSwitchPublisher()
options = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
           '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10,}

numbers = {'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten', 'eleven', 'twelve', '1', '2', '3',
           '4', '5', '6', '7', '8', '9', '10'}

# rostopic pub /nlp_out std_msgs/String "data: <ORDER>, [['steak', 1], ['fries', 1]], []"

class nlp_restaurant:
    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        self.response = ["", ""]
        self.callback = False
        self.image_switch_publisher = ImageSwitchPublisher()

    # Methods to prepare the received nlp data for further processing

    def split_response(self, data):
        """
        Removes all symbols that are not letters.
        :param: data: The response as a split list
        :return: A clean list
        """
        new_tmp = [n.strip() for n in data]
        real_msg = [re.sub('\W+', '', m) for m in new_tmp]
        return real_msg
    def split_number_word(self, input, numbers):
        """
        Because sometimes NLP returns the order and number in a combined string, we need
        to split the string into two parts.
        :param: input: The input string
        :param: numbers: The numbers to split
        :return: A clean list
        """
        print(input)

        result = []
        print("input", input)
        for input_str in input:
            strOrder = input_str[0]
            for number in numbers:
                if strOrder.startswith(number):
                    leftover = strOrder[len(number):]
                    tmpNum = options[number]
                    result.append((leftover, tmpNum))
        return result

    def save_order(self, data):
        """
        Creates a list of tuples based on the data input of NLP.
        :param: data: The list of received order
        :return: A list of tuples
        """
        tuple_order = [(x, options[y]) for x, y in zip(data, data[1:]) if y in options]
        for ord in tuple_order:
            for num in numbers:
                if num in ord[0]:
                    nlp_fallback = self.split_number_word(tuple_order, numbers)
                    return nlp_fallback
        return tuple_order

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


    def order_ready(self):
        """
        Confirmation from the bartender that order is ready.
        :param data: The response
        :return: Boolean
        """
        HeadFollowMotion(state='start').perform()
        TalkingMotion("Please confirm that the order is ready with a yes after my display changes.").perform()
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
        else:
            tries = 0
            while tries < 2:
                rospy.sleep(2.3)

                self.nlp_pub.publish("start")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

                start_time_rep = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time() - start_time_rep) == timeout:
                        rospy.logwarn("guest needs to repeat")
                        self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                        rospy.sleep(2)
                if self.response[0] == "<CONFIRM>":
                    return True
                else:
                    rospy.sleep(2)
                    tries += 1






    def took_order(self):
        """ Checks if the customer took their order.
        :param: data: Affirm or deny
        return: Bool
        """
        HeadFollowMotion(state='start').perform()

        TalkingMotion("Here is your order.").perform()
        rospy.sleep(2)
        TalkingMotion("Please say yes if you took your order after my display changes.").perform()
        rospy.sleep(2.5)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)
        print("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)


        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if int(time.time()) - start_time == timeout:
                rospy.logwarn("Guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
        self.callback = False
        if self.response[0] == "<CONFIRM>":
            return True
        elif self.response[0]=="<DENY<":
            return False

    def give_order(self, order: [(str, int)]):
        """
        Method to present the order to the bar personal.
        """
        HeadFollowMotion(state='start').perform()
        rospy.sleep(2)
        if len(order) == 1:
            TalkingMotion(f"Please prepare the order {order[0][1]} {order[0][0]}").perform()
            rospy.sleep(2)
        elif len(order) > 2:
            TalkingMotion("Please prepare the following order").perform()
            for n in order:
                TalkingMotion(f"{n[1]}{n[0]}").perform()

    def repeat_get_order(self, customer: CustomerDescription):
        """

        """
        global order
        HeadFollowMotion(state='start').perform()

        TalkingMotion("Please repeat your order when my display changes").perform()
        rospy.sleep(2.3)

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
        print(self.response)
        print(type(self.response))

        if self.response[0] == "<ORDER>":
            tmp = self.split_response(self.response)
            order = self.save_order(tmp)
            if order is not None:
                customer.set_order(order)
                self.confirm_order(customer=customer, order=order)
        else:
            tries = 0
            while tries <= 2:
                rospy.sleep(2.3)

                self.nlp_pub.publish("start")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

                start_time_rep = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time() - start_time_rep) == timeout:
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

    def confirm_order(self, customer: CustomerDescription, order: [(str, int)]):
        """
        Method to confirm an order with the current customer. If Toya did not understand the order
        correctly, the order process will be repeated.

        :param: order: The order of the current customer
        """
        HeadFollowMotion(state='start').perform()
        global repeat
        if len(order) == 1:

            TalkingMotion(f"Do you want to order {order[0][1]} {order[0][0]}?").perform()
            rospy.sleep(2)
            TalkingMotion("Please confirm with a yes or no after my display changes").perform()
            rospy.sleep(2.5)


            rospy.loginfo("nlp start")
            self.nlp_pub.publish("start listening")
            rospy.sleep(2)
            image_switch_publisher.pub_now(ImageEnum.TALK.value)

            start_time = time.time()
            while not self.callback:
                rospy.sleep(1)
                if int(time.time()) - start_time == timeout:
                    rospy.logwarn("Guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            self.callback = False
            if self.response[0] == "<CONFIRM>":
                print("I was here")
                return True
            elif self.response[0] == "<DENY>":
                self.repeat_get_order(customer=customer)
                return False


        else:
            TalkingMotion(f"Do you want to order the following items").perform()
            for n in order:
                 TalkingMotion(f"{n[1]} {n[0]} and").perform()
                 rospy.sleep(2)
            TalkingMotion("Confirm your order with a yes, after my display changes").perform()
            rospy.sleep(2.5)

            rospy.loginfo("nlp start")
            self.nlp_pub.publish("start listening")
            rospy.sleep(2)

            image_switch_publisher.pub_now(ImageEnum.TALK.value)
            start_time = time.time()
            while not self.callback:
                rospy.sleep(1)
                if int(time.time()) - start_time == timeout:
                    rospy.logwarn("Guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            self.callback = False
            if self.response[0] == "<CONFIRM>":
                repeat = False
                return True
            elif self.response[0] == "<DENY>":
                repeat = True
                self.repeat_get_order(customer=customer)
                return False


    def get_order(self, customer: CustomerDescription):
        """
        Method to order food if Toya successfully arrived at a customer.
        :param: customer: The customer that wants to oder something
        """
        global order
        HeadFollowMotion(state='start').perform()

        TalkingMotion("Welcome, what can I get for you?").perform()
        rospy.sleep(1.75)
        TalkingMotion("Please come close to me and order when my display changes").perform()
        rospy.sleep(2.5)


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
        print(self.response)
        print(type(self.response))

        if self.response[0] == "<ORDER>":
            tmp = self.split_response(self.response)
            order = self.save_order(tmp)
            if order is not None:
                customer.set_order(order)


        else:
            tries = 0
            while tries <= 2:
                rospy.sleep(2.3)

                self.nlp_pub.publish("start")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

                start_time_rep = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time() - start_time_rep) == timeout:
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



