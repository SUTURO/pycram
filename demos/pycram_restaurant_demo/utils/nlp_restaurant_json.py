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
        self.response = ["", [(str, int)]]
        self.confirmation = ""
        self.callback = False
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
                TalkingMotion(f"{n[0][1]} {n[0][0]} and").perform()
                txt_order += f"{n[0][1]} {n[0][0]}"
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

    # {"sentence": "I would like to order one banana and one apple .", "intent": "Order", "Item": {"value": "apple", "entity": "food", "propertyAttribute": [], "actionAttribute": [], "numberAttribute": ["one"]}}

    def parse_confirmation_string(self, json_string : str):
        print(json_string)
        try:
            parsed = json.loads(json_string)
            intent = parsed.get('intent')
            if intent == "affirm":
                self.confirmation = intent
            elif intent == "deny":
                self.confirmation = intent
        except (ValueError, SyntaxError, IndexError) as e:
            self.confirmation = "affirm"


    def parse_json_string(self, json_string: str):
        print(json_string)
        try:
            parsed_list = ast.literal_eval(json_string)
            parsed = json.loads(json_string)

            intent = parsed.get('intent')
            print("Intent", intent)
            if intent == "affirm":

                self.confirmation = intent
            elif intent == "deny":
                self.confirmation = intent
            entities = parsed.get('Item', {})
            print("Entities", )
            items = []
            amount = []
            order = []

            if intent == "Order":
                if isinstance(entities, dict) and 'value' in entities:
                    entities = {'item1': entities}

                for key, entity in entities.items():
                    print("\t", key, entity)
                    print(entity['value'])
                    print(entity.get('numberAttribute'))
                    item = entity.get('value')
                    num = entity.get('numberAttribute')
                    if num == ():
                        num = 1
                    elif isinstance(num[0], str):
                        print(type(options))
                        try:
                            num = options[num[0]]
                        except KeyError:
                            num = 1
                        print(num)
                    elif isinstance(num[0], int):
                        num = num[0]
                    items.append(item)
                    amount.append(num)

                order = list(zip(items, amount))
                self.response = [intent, order]


        except (ValueError, SyntaxError, IndexError) as e:
            print(f"Error parsing string: {e}")
            self.response = ["Order", ("Water", 1)]
