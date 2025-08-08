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
        # Variables for the NLP Publisher and Subscriber
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)

        self.response = [None, None]
        self.confirmation = None
        self.callback = False
        self.timeout = 15
        self.image_switch_publisher = ImageSwitchPublisher()
        self.text_image_switch_publisher = TextToImagePublisher()

    #---------NLP Handling-------------

    def data_cb(self, data):
        self.response = self.parse_json_string(data.data)
        self.confirmation = self.response[0]
        self.callback = True

    def parse_json_string(self, json_string: str):
        try:
            parsed = json.loads(json_string)
            intent = parsed.get('intent')
            entities = parsed.get('entities', [])
            order = []

            if intent == "Order":
                for entity in entities:
                    item = entity.get('value')
                    amount = entity.get('numberAttribute', [1])[0]
                    num = options.get(str(amount), 1)
                    order.append((item, num))
            return [intent, order]
        except Exception as e:
            rospy.logerr(f"Failed to parse JSON: {e}")
            return ["Order", [("water", 1)]]

    def _start_listening(self):
        rospy.loginfo("NLP start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

    def wait_for_callback(self, timeout=None):
        timeout = timeout or self.timeout
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if time.time() - start_time > timeout:
                return False
        return True

    def retry_nlp_attempts(self, max_retires=2):
        for _ in range(max_retires):
            self._start_listening()
            if self.wait_for_callback():
                self.callback = False
                if self.confirmation == "affirm":
                    return True
                elif self.confirmation == "deny":
                    return False
        return False


    #----------------------- Order Confirmation------------------
    def confirm_order(self, customer: CustomerDescription):
        HeadFollowMotion(state='start').perform()
        order = customer.order
        if len(order) == 1:
            self._confirm_single_item_order(order[0])
        else:
            self._confirm_multiple_items_order(order)

        if self.confirmation == "affirm":
            HeadFollowMotion(state='stop').perform
            return True
        elif self.confirmation =="deny":
            return False

        else:
            return self.retry_nlp_attempts()

    def _confirm_single_item_order(self, item):
        name, amount = item
        TalkingMotion(f"Do you want to order {amount} {name} ?").perform()
        self.text_image_switch_publisher.pub_now(f"order: {amount} {name}")
        rospy.sleep(2)
        TalkingMotion("Please confirm with a yes or a no after my display changes").perform()4
        rospy.sleep(2.5)

        self._start_listening()
        self.wait_for_callback()
        self.callback = False

    def _confirm_multiple_items_order(self, order):
        TalkingMotion("Do you want to order the following items").perform()
        txt_order = ""
        rospy.sleep(2)
        for item, amount in order:
            TalkingMotion(f"{amount} {item}").perform()
            txt_order += f"{amount} {item} "
            rospy.sleep(2)
        self.text_image_switch_publisher.pub_now(txt_order)
        TalkingMotion("Confirm your order with a yes or a no, after my display changes ").perform()
        rospy.sleep(2.2)
        self._start_listening()
        self.wait_for_callback()
        self.callback = False