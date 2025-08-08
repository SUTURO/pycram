import ast
import json
import time

import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.motion_designator import HeadFollowMotion, TalkingMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher, TextToImagePublisher

response = [None, None]
confirmation = None
callback = False
timeout = 15
# Due to the possibility that NLP will return the amount of the item, we have
# to use this dictionary to map the string versions to the corresponding int version
options = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
           '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10, }


class NLPRestaurant:
    """
    Class that stores the information about an order of a customer
    """

    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        rospy.sleep(2)
        self.order = None
        self.response = ["", [(str, int)]]
        self.confirmation = ""
        self.callback = False
        self.timeout = 15
        self.image_switch_publisher = ImageSwitchPublisher()
        self.text_image_switch_publisher = TextToImagePublisher()

    def data_cb(self, data):
        self.parse_json_string(data.data)
        self.callback = True

    def confirm_order(self, customer: CustomerDescription):
        """
        Method to confirm the order of a customer

        :param customer: The customer
        """
        HeadFollowMotion(state='start').perform()
        currentOrder = customer.order
        if len(currentOrder) == 1:
            TalkingMotion(f"Do you want to order {order[0][1]} {order[0][0]}").perform()
            self.text_image_switch_publisher.pub_now(f"order: {order[0][1]} {order[0][0]}")
            rospy.sleep(2)
            TalkingMotion("Please confirm with a yes or no after my display changes").perform()
            rospy.sleep(2.5)

            rospy.loginfo("nlp start")
            self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
            self.nlp_pub.publish("start listening")
            rospy.sleep(2.3)
            start_time = time.time()
            while not self.callback:
                rospy.sleep(1)
                if int(time.time()) - start_time == timeout+10:
                    rospy.logwarn("Guest needs to repeat")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

            self.callback = False

            if self.confirmation == "affirm":
                HeadFollowMotion(state='stop').perform()
                return True
            elif not self.confirmation == "deny":
                self.repeat_get_order(customer=customer)
                return False
            else:
                tries = 1
                while tries <= 2:
                    rospy.sleep(2.3)
                    self.nlp_pub.publish("start")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

                    start_time_rep = time.time()
                    while not self.callback:
                        rospy.sleep(1)
                        if int(time.time()) - start_time_rep == timeout:
                            rospy.logwarn("Guest needs to repeat")
                            self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                            rospy.sleep(2)
                    self.callback = False
                    if self.confirmation == "affirm":
                        HeadFollowMotion(state='stop').perform()
                        return True
                    elif not self.confirmation == "deny":
                        tries += 1
                        self.repeat_get_order(customer=customer)
                        return False
                    else:
                        tries += 1
        else:
            TalkingMotion("Do you want to order the following items").perform()
            txt_order = ""
            for n in currentOrder:
                TalkingMotion(f"{n[1]} {n[0]} and").perform()
                txt_order += f"{n[1]} {n[0]}"
                rospy.sleep(2)
            self.text_image_switch_publisher.pub_now(txt_order)
            TalkingMotion("Confirm your order with a yes, after my display changes").perform()
            rospy.sleep(2.5)
            self.image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)

            rospy.loginfo("nlp start")
            self.nlp_pub.publish("start listening")
            rospy.sleep(2)

            self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
            self.nlp_pub.publish("start listening")
            rospy.sleep(2.3)
            start_time = time.time()
            while not self.callback:
                rospy.sleep(1)
                if int(time.time()) - start_time == timeout:
                    rospy.logwarn("Guest needs to repeat")
                    self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            self.callback = False
            if self.confirmation == "affirm":
                HeadFollowMotion(state='stop').perform()
                return True
            elif not self.confirmation == "deny":
                self.repeat_get_order(customer=customer)
                return False
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
                    if self.confirmation == "affirm":
                        HeadFollowMotion(state='stop').perform()
                        return True
                    elif not self.confirmation =="deny":
                        self.repeat_get_order(customer=customer)
                        return False
                    else:
                        tries += 1

    def repeat_get_order(self, customer: CustomerDescription):
        """
        Method for the case if HSR did not understood the order correctly.
        :param: customer: The customer associated with the order

        """
        global order
        HeadFollowMotion(state='start').perform()
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        TalkingMotion("Please repeat your order when my display changes").perform()
        rospy.sleep(2.3)

        print("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if int(time.time()) - start_time == timeout:
                rospy.logwarn("Guest needs to repeat")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                rospy.sleep(2)
        self.callback = False
        if self.response[0] == "Order":
            order =self.response[1]

            if order is not None:
                customer.set_order(order)
                self.confirm_order(customer= customer)
        else:
            tries = 0
            while tries <= 2:
                rospy.sleep(2.3)
                self.nlp_pub.publish("start")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

                start_time_rep = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time()) - start_time_rep == timeout:
                        rospy.logwarn("Guest needs to repeat")
                        self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                        rospy.sleep(2)
                self.callback = False
                if self.response[0] == "Order":
                    tmp_order = self.response[1]
                    if tmp_order is not None:
                        customer.set_order(tmp_order)
                        break
                    else:
                        print(tries)
                        tries += 1



    def get_order(self, customer: CustomerDescription):
        """Method to order food if Toya successfully arrived at a customer.
        :param customer: The customer that will be associated with the order"""
        global order
        HeadFollowMotion(state='start').perform()

        TalkingMotion("Welcome, what can I get for you?").perform()
        rospy.sleep(1.75)
        TalkingMotion("Please come close to me and order when my display changes").perform()
        rospy.sleep(2.5)
        # Signal to start listening
        print("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2.5)


        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
        # We have to wait a little bit for NLP
        start_time = time.time()
        tries = 0
        while not self.callback:
            rospy.sleep(1)

            if int(time.time()) - start_time == timeout:
                print("Guest needs to repeat")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                rospy.sleep(2)
        self.callback = False
        print(self.response)
        # Check if we received the desired information
        if self.response[0] == "Order":
            order = self.response[1]
            if order is not None:
                customer.set_order(order)
            else:
                customer.set_order([('water', 1)])
        else:
            tries = 0
            while tries <= 2:
                rospy.sleep(2.3)
                self.nlp_pub.publish("Start again")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

                start_time_rep = time.time()
                while not self.callback:
                    rospy.sleep(1)
                    if int(time.time()) - start_time_rep == timeout:
                        rospy.logwarn("Guest needs to repeat")
                        self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                        rospy.sleep(2)
                        tries += 1
                self.callback = False
                if self.response[0] == "Order":
                    tmp_order = self.response[1]
                    if tmp_order is not None:
                        customer.set_order(tmp_order)
                        break
                    else:
                        tries += 1

    def wait_for_callback(self, timeout=None) -> bool:
        """
        Waits for the NLP callback to be triggered within the timeout.
        :return: True if callback was triggered, False otherwise
        """
        timeout = timeout or self.timeout
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if time.time() - start_time > timeout:
                return False
        return True
    def retry_nlp_attempts(self, max_retries=2) -> bool:
        """
        Retry NLP listening and parsing for confirmation or order.
        Returns True if successful confirmation or order received.
        """
        for _ in range(max_retries):
            self._start_listening()
            if self.wait_for_callback():
                self.callback = False
                if self.con
    def _handle_nlp_response_get_order(self, timeout = 15):
        """
        Helper Method to handle the normal ordering step
        """
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if time.time() - start_time > timeout:
                return
    def _start_listening(self):
        """
        Helper Method to start the NLP side of this challenge
        """
        print("NLP start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

    def _guide_user_to_order(self):
        """
        Helper metho to make the HSR say the needed phrases
        """
        HeadFollowMotion(state='start').perform()
        rospy.sleep(2)

        phrases = [
            "Welcome, what can I get for you?",
            "Please come close to me and order when my display changes"
        ]
        for phrase in phrases:
            TalkingMotion(phrase).perform()
            rospy.sleep(2.8)

    def _confirm_order_with_guest(self, order: [(str, int)]):
        """
        Helper Method to confirm the received order with the customer.
        :param: order: The order that the customer needs to confirm
        """
        HeadFollowMotion(state='start').perform()
        rospy.sleep(2)
        prepared_order = [str]
        for n in order:
            item = n[0]
            amount = n[1]
            prepared_order.append(str(amount) + " " + item )
        TalkingMotion("Please confirm with a yes or no the following order: ").perform()
        rospy.sleep(2.5)
        for entity in order:
            TalkingMotion(entity).perform()
            rospy.sleep(2)

    def _prepare_order_for_text(self, order: [(str, int)]):
        """
        Helper Method to prepare the order to be published on the display of the HSR
        :param: order: The received order of the customer
        """
        tmp_ord = [str]
        for n in order:
            item = n[0]
            amount = n[1]
            tmp_str = str(amount) + " " + item
            tmp_ord.append(tmp_str)
        self.text_image_switch_publisher.pub_now(tmp_ord)
        rospy.sleep(2.5)



### EXAMPLE SENTENCE ###
    #{"sentence": "I would like to order one fry and one burger .",
    # "intent": "Order",
    # "entities": [{"role": "Item", "value": "fry", "entity": "food", "propertyAttribute": [], "actionAttribute": [], "numberAttribute": ["one"]},
    # {"role": "Item", "value": "burger", "entity": "food", "propertyAttribute": [], "actionAttribute": [], "numberAttribute": ["one"]}]}

    def parse_json_string(self, json_string: str):
        """
        Method do transform the received data from NLP to a workable
        list of tuples representing the order of a customer or the confirmation of
        the order.
        :param: json_string: the received data from NLP

        """
        print(json_string)
        try:
            parsed = json.loads(json_string)

            intent = parsed.get('intent')
            print("Intent", intent)
            if intent == "affirm":

                self.confirmation = intent
            elif intent == "deny":
                self.confirmation = intent
            entities = parsed.get('entities')
            order = []
            if intent == "Order":

                for entity in entities:
                    item = entity.get('value')
                    amount = entity.get('numberAttribute')
                    num = 0
                    if amount[0] == "":
                        num = 1
                    elif isinstance(amount[0], str):
                        try:
                            num = options[amount[0]]
                        except KeyError:
                            num = 1
                    elif isinstance(amount[0], int):
                            num = amount[0]
                    order.append((item,num))
            self.response = [intent, order]



        except (ValueError, SyntaxError, IndexError) as e:
            print(f"Error parsing string: {e}")
            self.response = ["Order", ("Water", 1)]
