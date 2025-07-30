import ast
import json
import time
from typing import List, Dict, Any
import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.motion_designator import HeadFollowMotion, TalkingMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher, TextToImagePublisher

response = []
confirmation = []
callback = False
timeout = 10

text_to_image_pub = TextToImagePublisher()
image_switch_pub = ImageSwitchPublisher()
# Hot fix because NLP returns the amount sometimes as a written out string or the int as a string
options = {
    'one' :1,
    'two':2,
    'three': 3,
    'four': 4,
    'five': 5,
    'six':6,
    'seven':7,
    'eight': 8,
    'nine': 9,
    'ten':10,
    '1': 1,
    '2': 2,
    '3': 3,
    '4': 4,
    '5': 5,
    '6': 6,
    '7': 7,
    '8': 8,
    '9': 9,
    '10': 10
}
# Similar to options, we have to check if these numbers are part of the received data callback
numbers = {
    'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten',
    '1', '2', '3', '4', '5', '6', '7', '8', '9', '10'
}
class RestaurantManager():
    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self._data_callback)
        self.response = None
        self.callback = False
        self.image_switch_publisher = ImageSwitchPublisher()
        self.text_to_image_pub = TextToImagePublisher()

    def parse_nlp_response(self, data:str):
        print(data)
        try:
            return json.loads(data)
        except:
            rospy.logwarn("Failed to parse NLP")
            return None

    def _data_callback(self, data: String):
        """
        Receives the data from NLP and dumps it into a JSON, to optimize working with it.
        :param data: NLP response

        """

        try:
            self.response = self.parse_nlp_response(data.data)
            print(self.response)
            print("Type " , type(self.response))
            print(self.response.keys())
            if self.response:
                self.callback = True
                rospy.loginfo("Received NLP data")
            else:
                rospy.logwarn("Received empty")
        except Exception as e:
            rospy.logerr(f"Error processing NLP {e}")
            self.response = None
            self.callback = False

    def word_to_number(self, word: str) -> int:
        """
        Fallback method if NLP returns the int in a string format,
        :param: word: The number
        :return: The number as an int
        """
        try:
            intVer = options.get(word.lower())
        except Exception as e:
            rospy.logwarn("Number was not in the options")
            return 1
        return intVer
    def extract_order_from_response(self, msg: Dict[str, Any]):
        """
        Extracts the required data from the NLP feedback.
        :param: nlp_data : Received data
        'return: List of (str, int) for the order
        """

        order_item = []
        msgList = msg[0]
        print(msg)
        if msgList['intent'] == 'Order':
            for key, entity in msgList['entities'].items():
                value = entity['value']
                number = entity['numberAttribute']
                print("type of number ", type(number))
                if not number:
                    number = 1
                elif not isinstance(number, int):
                    print("hellloo")
                    number = options.get(number[0])
                    print(options.get(number))

            order_item = list(zip(value, number))
        # for values in list_order.values():
        #     print(values)
        #     print(values["value"])
        #     list_entity.append(values["value"])
        #     tmp_num = values["numberAttribute"]
        #     if tmp_num == ():
        #         list_num.append(1)
        #     else:
        #         tmp = tmp_num[0]
        #         real_int = options.get(tmp)
        #         list_num.append(real_int)
        print(order_item)
        return order_item

    def get_order(self, customer:CustomerDescription):
        """
        Method to take the order of a customer.
        :param: customer: The customer that will be associated with this order
        """

        HeadFollowMotion(state='start').perform()

        TalkingMotion("Welcome, what can I get for you?").perform()
        rospy.sleep(1.75)
        TalkingMotion("Please come close to me and order if my display changes.").perform()
        rospy.sleep(2.5)

        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not self.callback and (time.time() - start_time) < timeout:
            rospy.sleep(0.1)

        if not self.callback:
            rospy.logwarn("No response received from NLP")
            self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            return

        self.callback = False

        if self.response and self.response.get('intent') == 'Order':
            order_items = self.extract_order_from_response(self.response)
            print(order_items)
            if order_items:
                customer.set_order(order_items)
                self.confirm_order(customer, order_items)
            else:
                rospy.logwarn("No order was found in the response")

        else:
            rospy.logwarn("Response was not an order")
            self.repeat_get_order(customer, 1)

    def confirm_order(self, customer: CustomerDescription, order:List[tuple]):
        """Confirms the order
            :param: customer: The customer that confirms the order
            :param: order: The order to confirm
        """

        HeadFollowMotion(state='start').perform()

        if len(order) == 1:
            item, quantity = order[0]
            TalkingMotion(f"Do you want to order {quantity} {item}")
            text_to_image_pub.pub_now(f"{quantity} {item}")
        else:
            TalkingMotion("Do you want to order the following items?").perform()
            orderTest = "Do you want to order "
            for item, quanity in order:
                TalkingMotion(f"{quanity} {item} and").perform()
                rospy.sleep(1)
                orderTest+= f"{quanity} {item}"
            text_to_image_pub.pub_now(orderTest).perform()

        rospy.sleep(2)
        image_switch_pub.pub_now(ImageEnum.GENERATED_TEXT.value)
        TalkingMotion("Please confirm with a yes or no when my display changes").perform()
        rospy.sleep(2.3)

        self.nlp_pub.publish("start listening")
        rospy.sleep(2)
        image_switch_pub.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not self.callback and (time.time() - start_time) > timeout:
            rospy.sleep(0.1)

        if not self.callback:
            rospy.logwarn("No confirmation received")
            return False

        self.callback = False

    def repeat_get_order(self, customer: CustomerDescription, tries: int):
        while tries <= 3:
            HeadFollowMotion(state='start').perform()

            TalkingMotion("Please repeat your order").perform()
            rospy.sleep(1.75)
            TalkingMotion("Please come close to me and order if my display changes.").perform()
            rospy.sleep(2.5)

            self.nlp_pub.publish("start listening")
            rospy.sleep(2.3)
            self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

            start_time = time.time()
            while not self.callback and (time.time() - start_time) < timeout:
                rospy.sleep(0.1)

            if not self.callback:
                rospy.logwarn("No response received from NLP")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                return

            self.callback = False

            if self.response and self.response.get('intent') == 'Order':
                order_items = self.extract_order_from_response(self.response)
                print(order_items)
                if order_items:
                    customer.set_order(order_items)
                    self.confirm_order(customer, order_items)
                else:
                    rospy.logwarn("No order was found in the response")
            else:
                rospy.logwarn("Response was not an order")
            tries += 1
        if tries == 3:
            TalkingMotion("Do you want to order a water?").perform()