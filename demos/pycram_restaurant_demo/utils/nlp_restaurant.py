import ast
import json
import time
from typing import List

import rospy
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum
from pycram.designators.action_designator import DetectAction
from pycram.designators.motion_designator import TalkingMotion, HeadFollowMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher, TextToImagePublisher
import re

response = [None, None]
confirmation = [None]
callback = False
timeout = 10
global repeat
repeat = False
text_to_image_publisher = TextToImagePublisher()
image_switch_publisher = ImageSwitchPublisher()
options = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
           '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10, }

numbers = {'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten', 'eleven', 'twelve', '1', '2',
           '3',
           '4', '5', '6', '7', '8', '9', '10'}


# rostopic pub /nlp_out std_msgs/String "data: <ORDER>, [['steak', 1], ['fries', 1]], []"

class nlp_restaurant:

    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
        rospy.sleep(2)
        self.response = ["", ""]
        self.callback = False
        self.image_switch_publisher = ImageSwitchPublisher()
        self.text_switch_publisher = TextToImagePublisher()

    def data_cb(self, data):
        self.image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.parse_json_string(data.data)
        self.callback = True


    def parse_json_string(self, json_string: str):

        print(json_string)
        parsed = json.loads(json_string)

        intent = parsed.get("intent")
        items = parsed.get("Item")
        print(items)
        if intent == "Order":
            for entities in items.items():
                print(entities.get("value"))
                print(entities.get("numberAttribute"))

    def get_order(self, customer: CustomerDescription):
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
        tries = 0
        while not self.callback and tries < 2:
            rospy.sleep(1)
            if int(time.time() - start_time) == timeout:
                rospy.logwarn("guest needs to repeat")
                print("listen again")
                self.nlp_pub.publish("start listening")
                start_time = time.time()
                rospy.sleep(1)
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                tries += 1
        self.callback = False
        print(self.response)


    # def __init__(self):
    #     self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
    #     self.sub_nlp = rospy.Subscriber("nlp_out", String, self.data_cb)
    #     self.response = ["", ""]
    #     self.response = []
    #     self.callback = False
    #     self.image_switch_publisher = ImageSwitchPublisher()
    #
    # # Methods to prepare the received nlp data for further processing
    #
    # def split_response(self, data):
    #     """
    #     Removes all symbols that are not letters.
    #     :param: data: The response as a split list
    #     :return: A clean list
    #     """
    #     new_tmp = [n.strip() for n in data]
    #     real_msg = [re.sub('\W+', '', m) for m in new_tmp]
    #     return real_msg
    #
    # def get_order_data(self, data):
    #
    #     """
    #
    #
    #     Method to extract the necessary data (entity values and numberAttributes) from the received Data.
    #     Only useable if the gpsr nlp script is running
    #     :param: data: NLP Response
    #     :return: A list of tuples
    #
    #
    #     """
    #
    #     order_list = []
    #
    #     msg = ast.literal_eval(self.response2)
    #
    #     print("Messages", msg.keys())
    #
    #     if msg['intent'] == "Order":
    #
    #         list_order = msg['Item']
    #
    #         print(list_order)
    #
    #         tmp_entity = list_order['value']
    #
    #         print(tmp_entity)
    #
    #         tmp_num = list_order['numberAttribute']
    #         list_num = []
    #
    #         list_entity = []
    #
    #         list_entity.append(tmp_entity)
    #         if tmp_num == ():
    #             list_num.append(1)
    #         else:
    #
    #             tmp = tmp_num[0]
    #
    #             real_int = options.get(tmp)
    #
    #             list_num.append(real_int)
    #
    #         order_list = list(zip(list_entity, list_num))
    #
    #         print("order", order_list)
    #         return order_list
    #
    # def split_number_word(self, input, numbers):
    #     """
    #     Because sometimes NLP returns the order and number in a combined string, we need
    #     to split the string into two parts.
    #     :param: input: The input string
    #     :param: numbers: The numbers to split
    #     :return: A clean list
    #     """
    #
    #     result = []
    #     print("input", input)
    #     for input_str in input:
    #         strOrder = input_str[0]
    #         if input_str[1] != 1:
    #             result.append((input_str[0], input_str[1]))
    #         for number in numbers:
    #             if strOrder.startswith(number):
    #                 leftover = strOrder[len(number):]
    #                 tmpNum = options[number]
    #                 if (leftover, tmpNum) not in result:
    #                     result.append((leftover, tmpNum))
    #
    #     print(result)
    #     return result
    #
    # def save_order(self, data):
    #     """
    #     Creates a list of tuples based on the data input of NLP.
    #     :param: data: The list of received order
    #     :return: A list of tuples
    #     """
    #     tuple_order = [(x, options[y]) for x, y in zip(data, data[1:]) if y in options]
    #     for ord in tuple_order:
    #         for num in numbers:
    #             if num in ord[0]:
    #                 nlp_fallback = self.split_number_word(tuple_order, numbers)
    #                 return nlp_fallback
    #     return tuple_order
    #
    # def data_cb(self, data):
    #     """
    #     function to receive data from nlp via /nlp_out topic
    #     """
    #     image_switch_publisher = ImageSwitchPublisher()
    #     image_switch_publisher.pub_now(ImageEnum.HI.value)
    #
    #     self.response2 = data.data
    #     self.response = data.data.split(",")
    #     for ele in self.response:
    #         print(ele)
    #         ele.strip()
    #     self.response.append("None")
    #     print(self.response)
    #     self.callback = True
    #
    # def order_ready(self):
    #     """
    #     Confirmation from the bartender that order is ready.
    #     :param data: The response
    #     :return: Boolean
    #     """
    #     HeadFollowMotion(state='start').perform()
    #     TalkingMotion("Please confirm that the order is ready with a yes after my display changes.").perform()
    #     rospy.sleep(2)
    #     test = self.get_order_data(self.response)
    #     print(test)
    #     rospy.loginfo("nlp start")
    #     self.nlp_pub.publish("start listening")
    #     rospy.sleep(2)
    #     self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
    #
    #     start_time = time.time()
    #     while not self.callback:
    #         rospy.sleep(1)
    #         if int(time.time()) - start_time == timeout:
    #             rospy.loginfo("Please repeat")
    #             self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #     self.callback = False
    #     print(self.response)
    #     if self.response[0] == "<CONFIRM>":
    #         return True
    #     elif self.response[0] == "<DENY>":
    #         rospy.sleep(4)
    #     else:
    #         tries = 0
    #         while tries < 2:
    #             rospy.sleep(2.3)
    #
    #             self.nlp_pub.publish("start")
    #             self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #
    #             start_time_rep = time.time()
    #             while not self.callback:
    #                 rospy.sleep(1)
    #                 if int(time.time() - start_time_rep) == timeout:
    #                     rospy.logwarn("guest needs to repeat")
    #                     self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #                     rospy.sleep(2)
    #             if self.response[0] == "<CONFIRM>":
    #                 return True
    #             else:
    #                 rospy.sleep(2)
    #                 tries += 1
    #
    # def took_order(self):
    #     """ Checks if the customer took their order.
    #     :param: data: Affirm or deny
    #     return: Bool
    #     """
    #     HeadFollowMotion(state='start').perform()
    #
    #     TalkingMotion("Here is your order.").perform()
    #     rospy.sleep(2)
    #     TalkingMotion("Please say yes if you took your order after my display changes.").perform()
    #     rospy.sleep(2.5)
    #     image_switch_publisher.pub_now(ImageEnum.TALK.value)
    #     print("nlp start")
    #     self.nlp_pub.publish("start listening")
    #     rospy.sleep(2.3)
    #     self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
    #
    #     start_time = time.time()
    #     while not self.callback:
    #         rospy.sleep(1)
    #         if int(time.time()) - start_time == timeout:
    #             rospy.logwarn("Guest needs to repeat")
    #             image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #     self.callback = False
    #     if self.response[0] == "<CONFIRM>":
    #         return True
    #     elif self.response[0] == "<DENY<":
    #         return False
    #
    # def give_order(self, order: [(str, int)]):
    #     """
    #     Method to present the order to the bar personal.
    #     """
    #     HeadFollowMotion(state='start').perform()
    #     rospy.sleep(2)
    #     if len(order) == 1:
    #         TalkingMotion(f"Please prepare the order {order[0][1]} {order[0][0]}").perform()
    #         rospy.sleep(2)
    #     elif len(order) > 2:
    #         TalkingMotion("Please prepare the following order").perform()
    #         for n in order:
    #             TalkingMotion(f"{n[1]}{n[0]}").perform()
    #
    # def repeat_get_order(self, customer: CustomerDescription):
    #     """
    #
    #     """
    #     global order
    #     HeadFollowMotion(state='start').perform()
    #
    #     TalkingMotion("Please repeat your order when my display changes").perform()
    #     rospy.sleep(2.3)
    #
    #     print("nlp start")
    #     self.nlp_pub.publish("start listening")
    #     rospy.sleep(2.3)
    #     self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
    #     start_time = time.time()
    #     while not self.callback:
    #         rospy.sleep(1)
    #
    #         if int(time.time() - start_time) == timeout:
    #             print("guest needs to repeat")
    #             image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #             rospy.sleep(2)
    #             customer.set_order([('water', 1)])
    #             break
    #
    #     self.callback = False
    #     print(self.response)
    #     print(type(self.response))
    #
    #     if self.response[0] == "<ORDER>":
    #         tmp = self.split_response(self.response)
    #         order = self.save_order(tmp)
    #         print(order)
    #         if order is not None:
    #             customer.set_order(order)
    #             self.confirm_order(customer=customer, order=order)
    #     else:
    #         tries = 0
    #         while tries <= 2:
    #             rospy.sleep(2.3)
    #
    #             self.nlp_pub.publish("start")
    #             self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #
    #             start_time_rep = time.time()
    #             while not self.callback:
    #                 rospy.sleep(1)
    #                 if int(time.time() - start_time_rep) == timeout:
    #                     rospy.logwarn("guest needs to repeat")
    #                     self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #                     rospy.sleep(2)
    #
    #                     customer.set_order([('water', 1)])
    #
    #             self.callback = False
    #             if self.response[0] == "<ORDER>":
    #                 tmp_rep = self.split_response(self.response)
    #                 order_rep = self.save_order(tmp_rep)
    #                 if order_rep is not None:
    #                     customer.set_order(order_rep)
    #
    #                     break
    #                 else:
    #                     print(tries)
    #                     tries += 1
    #
    # def confirm_order(self, customer: CustomerDescription, order: [(str, int)]):
    #     """
    #     Method to confirm an order with the current customer. If Toya did not understand the order
    #     correctly, the order process will be repeated.
    #
    #     :param: order: The order of the current customer
    #     """
    #     HeadFollowMotion(state='start').perform()
    #     global repeat
    #     if len(order) == 1:
    #
    #         TalkingMotion(f"Do you want to order {order[0][1]} {order[0][0]}?").perform()
    #         text_to_image_publisher.pub_now(f"order: {order[0][1]} {order[0][0]}")
    #         rospy.sleep(2)
    #         image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
    #         TalkingMotion("Please confirm with a yes or no after my display changes").perform()
    #         rospy.sleep(2.5)
    #
    #         rospy.loginfo("nlp start")
    #         self.nlp_pub.publish("start listening")
    #         rospy.sleep(2)
    #         image_switch_publisher.pub_now(ImageEnum.TALK.value)
    #
    #         start_time = time.time()
    #         while not self.callback:
    #             rospy.sleep(1)
    #             if int(time.time()) - start_time == timeout:
    #                 rospy.logwarn("Guest needs to repeat")
    #                 image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #         self.callback = False
    #         if self.response[0] == "<CONFIRM>":
    #             HeadFollowMotion(state='stop').perform()
    #             return True
    #         elif self.response[0] == "<DENY>":
    #             self.repeat_get_order(customer=customer)
    #             return False
    #         else:
    #             tries = 0
    #             while tries <= 2:
    #                 rospy.sleep(2.3)
    #
    #                 self.nlp_pub.publish("start")
    #                 self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #
    #                 start_time_rep = time.time()
    #                 while not self.callback:
    #                     rospy.sleep(1)
    #                     if int(time.time() - start_time_rep) == timeout:
    #                         rospy.logwarn("guest needs to repeat")
    #                         self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #                         rospy.sleep(2)
    #                 self.callback = False
    #                 if self.response[0] == "<CONFIRM>":
    #                     HeadFollowMotion(state='stop').perform()
    #                     return True
    #                 elif self.response[0] == "<DENY>":
    #                     self.repeat_get_order(customer=customer)
    #                     return False
    #                 else:
    #                     tries += 1
    #
    #
    #     else:
    #         TalkingMotion(f"Do you want to order the following items").perform()
    #         txt_order = ""
    #         for n in order:
    #             TalkingMotion(f"{n[1]} {n[0]} and").perform()
    #             txt_order += f"{n[1]} {n[0]}"
    #             rospy.sleep(2)
    #         text_to_image_publisher.pub_now(txt_order)
    #         TalkingMotion("Confirm your order with a yes, after my display changes").perform()
    #         rospy.sleep(2.5)
    #         image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
    #
    #         rospy.loginfo("nlp start")
    #         self.nlp_pub.publish("start listening")
    #         rospy.sleep(2)
    #
    #         image_switch_publisher.pub_now(ImageEnum.TALK.value)
    #         start_time = time.time()
    #         while not self.callback:
    #             rospy.sleep(1)
    #             if int(time.time()) - start_time == timeout:
    #                 rospy.logwarn("Guest needs to repeat")
    #                 image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #         self.callback = False
    #         if self.response[0] == "<CONFIRM>":
    #             HeadFollowMotion(state='stop').perform()
    #             return True
    #         elif self.response[0] == "<DENY>":
    #             self.repeat_get_order(customer=customer)
    #             return False
    #         else:
    #             tries = 0
    #             while tries <= 2:
    #                 rospy.sleep(2.3)
    #
    #                 self.nlp_pub.publish("start")
    #                 self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #
    #                 start_time_rep = time.time()
    #                 while not self.callback:
    #                     rospy.sleep(1)
    #                     if int(time.time() - start_time_rep) == timeout:
    #                         rospy.logwarn("guest needs to repeat")
    #                         self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #                         rospy.sleep(2)
    #                 self.callback = False
    #                 if self.response[0] == "<CONFIRM>":
    #                     HeadFollowMotion(state='stop').perform()
    #                     return True
    #                 elif self.response[0] == "<DENY>":
    #                     self.repeat_get_order(customer=customer)
    #                     return False
    #                 else:
    #                     tries += 1
    #
    # def get_order(self, customer: CustomerDescription):
    #     """
    #     Method to order food if Toya successfully arrived at a customer.
    #     :param: customer: The customer that wants to oder something
    #     """
    #     global order
    #     HeadFollowMotion(state='start').perform()
    #
    #     TalkingMotion("Welcome, what can I get for you?").perform()
    #     rospy.sleep(1.75)
    #     TalkingMotion("Please come close to me and order when my display changes").perform()
    #     rospy.sleep(2.5)
    #
    #     print("nlp start")
    #     self.nlp_pub.publish("start listening")
    #     rospy.sleep(2.3)
    #     self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
    #     start_time = time.time()
    #     while not self.callback:
    #         rospy.sleep(1)
    #
    #         if int(time.time() - start_time) == timeout:
    #             print("guest needs to repeat")
    #             image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #             rospy.sleep(2)
    #
    #     self.callback = False
    #     print(self.response)
    #     print(type(self.response))
    #
    #     if self.response[0] == "<ORDER>":
    #         tmp = self.split_response(self.response)
    #         order = self.save_order(tmp)
    #         if order is not None:
    #             customer.set_order(order)
    #
    #
    #     else:
    #         tries = 0
    #         while tries <= 2:
    #             rospy.sleep(2.3)
    #
    #             self.nlp_pub.publish("start")
    #             self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #
    #             start_time_rep = time.time()
    #             while not self.callback:
    #                 rospy.sleep(1)
    #                 if int(time.time() - start_time_rep) == timeout:
    #                     rospy.logwarn("guest needs to repeat")
    #                     self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    #                     rospy.sleep(2)
    #
    #             self.callback = False
    #             if self.response[0] == "<ORDER>":
    #                 tmp_rep = self.split_response(self.response)
    #                 order_rep = self.save_order(tmp_rep)
    #                 if order_rep is not None:
    #                     customer.set_order(order_rep)
    #
    #                     break
    #                 else:
    #                     print(tries)
    #                     tries += 1

# import ast
# import json
# import time
# from typing import List, Dict, Any
# import rospy
# from std_msgs.msg import String
#
# from pycram.datastructures.enums import ImageEnum
# from pycram.designators.motion_designator import HeadFollowMotion, TalkingMotion
# from pycram.designators.object_designator import CustomerDescription
# from pycram.utilities.robocup_utils import ImageSwitchPublisher, TextToImagePublisher
#
# response = []
# confirmation = []
# callback = False
# timeout = 10
#
# text_to_image_pub = TextToImagePublisher()
# image_switch_pub = ImageSwitchPublisher()
# # Hot fix because NLP returns the amount sometimes as a written out string or the int as a string
# options = {
#     'one' :1,
#     'two':2,
#     'three': 3,
#     'four': 4,
#     'five': 5,
#     'six':6,
#     'seven':7,
#     'eight': 8,
#     'nine': 9,
#     'ten':10,
#     '1': 1,
#     '2': 2,
#     '3': 3,
#     '4': 4,
#     '5': 5,
#     '6': 6,
#     '7': 7,
#     '8': 8,
#     '9': 9,
#     '10': 10
# }
# # Similar to options, we have to check if these numbers are part of the received data callback
# numbers = {
#     'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten',
#     '1', '2', '3', '4', '5', '6', '7', '8', '9', '10'
# }
# class RestaurantManager():
#     def __init__(self):
#         self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
#         self.sub_nlp = rospy.Subscriber("nlp_out", String, self._data_callback)
#         self.response = None
#         self.callback = False
#         self.image_switch_publisher = ImageSwitchPublisher()
#         self.text_to_image_pub = TextToImagePublisher()
#
#     def parse_nlp_response(self, data:str):
#         print(data)
#         try:
#             return json.loads(data)
#         except:
#             rospy.logwarn("Failed to parse NLP")
#             return None
#
#     def _data_callback(self, data: String):
#         """
#         Receives the data from NLP and dumps it into a JSON, to optimize working with it.
#         :param data: NLP response
#
#         """
#
#         try:
#             self.response = self.parse_nlp_response(data.data)
#             print(self.response)
#             print("Type " , type(self.response))
#             print(self.response.keys())
#             if self.response:
#                 self.callback = True
#                 rospy.loginfo("Received NLP data")
#             else:
#                 rospy.logwarn("Received empty")
#         except Exception as e:
#             rospy.logerr(f"Error processing NLP {e}")
#             self.response = None
#             self.callback = False
#
#     def word_to_number(self, word: str) -> int:
#         """
#         Fallback method if NLP returns the int in a string format,
#         :param: word: The number
#         :return: The number as an int
#         """
#         try:
#             intVer = options.get(word.lower())
#         except Exception as e:
#             rospy.logwarn("Number was not in the options")
#             return 1
#         return intVer
#     def extract_order_from_response(self, msg: Dict[str, Any]):
#         """
#         Extracts the required data from the NLP feedback.
#         :param: nlp_data : Received data
#         'return: List of (str, int) for the order
#         """
#
#         global number
#         global value
#         order_item = []
#         numbers = []
#         items = []
#         print(msg)
#         if msg['intent'] == 'Order':
#             for entity in msg['Item'].items():
#                 print("entitiy", entity)
#                 if entity[0] == 'value':
#                     value = entity[1]
#                     items.append(value)
#                 if entity[0] == 'numberAttribute':
#                     number = entity[1]
#
#                     if number == []:
#                         number = 1
#                     elif not isinstance(number, int):
#                         print("hellloo")
#                         print(number)
#
#                         number = options[number[0]]
#                        # print(options.get(number))
#                         numbers.append(number)
#
#             order_item = list(zip(items, numbers))
#         # for values in list_order.values():
#         #     print(values)
#         #     print(values["value"])
#         #     list_entity.append(values["value"])
#         #     tmp_num = values["numberAttribute"]
#         #     if tmp_num == ():
#         #         list_num.append(1)
#         #     else:
#         #         tmp = tmp_num[0]
#         #         real_int = options.get(tmp)
#         #         list_num.append(real_int)
#         print(order_item)
#         return order_item
#
#     def get_order(self, customer:CustomerDescription):
#         """
#         Method to take the order of a customer.
#         :param: customer: The customer that will be associated with this order
#         """
#
#         HeadFollowMotion(state='start').perform()
#
#         TalkingMotion("Welcome, what can I get for you?").perform()
#         rospy.sleep(1.75)
#         TalkingMotion("Please come close to me and order if my display changes.").perform()
#         rospy.sleep(2.5)
#
#         self.nlp_pub.publish("start listening")
#         rospy.sleep(2.3)
#         self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
#
#         start_time = time.time()
#         while not self.callback and (time.time() - start_time) < timeout:
#             rospy.sleep(0.1)
#
#         if not self.callback:
#             rospy.logwarn("No response received from NLP")
#             self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
#             return
#
#         self.callback = False
#
#         if self.response and self.response.get('intent') == 'Order':
#             order_items = self.extract_order_from_response(self.response)
#             print(order_items)
#             if order_items:
#                 customer.set_order(order_items)
#                 self.confirm_order(customer, order_items)
#             else:
#                 rospy.logwarn("No order was found in the response")
#
#         else:
#             rospy.logwarn("Response was not an order")
#             self.repeat_get_order(customer, 1)
#
#     def confirm_order(self, customer: CustomerDescription, order:List[tuple]):
#         """Confirms the order
#             :param: customer: The customer that confirms the order
#             :param: order: The order to confirm
#         """
#
#         HeadFollowMotion(state='start').perform()
#
#
#
#         rospy.sleep(2)
#         image_switch_pub.pub_now(ImageEnum.GENERATED_TEXT.value)
#         TalkingMotion("Please confirm with a yes or no when my display changes").perform()
#         rospy.sleep(2.3)
#
#         self.nlp_pub.publish("start listening")
#         rospy.sleep(2)
#         image_switch_pub.pub_now(ImageEnum.TALK.value)
#
#         start_time = time.time()
#         while not self.callback and (time.time() - start_time) > timeout:
#             rospy.sleep(0.1)
#
#         if self.response['intent'] == "affirm":
#             HeadFollowMotion(state='stop').perform()
#
#             return True
#
#         if not self.callback:
#             rospy.logwarn("No confirmation received")
#             HeadFollowMotion(state='stop').perform()
#
#             return False
#
#         self.callback = False
#
#     def repeat_get_order(self, customer: CustomerDescription, tries: int):
#         while tries <= 3:
#             HeadFollowMotion(state='start').perform()
#
#             TalkingMotion("Please repeat your order").perform()
#             rospy.sleep(1.75)
#             TalkingMotion("Please come close to me and order if my display changes.").perform()
#             rospy.sleep(2.5)
#
#             self.nlp_pub.publish("start listening")
#             rospy.sleep(2.3)
#             self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
#
#             start_time = time.time()
#             while not self.callback and (time.time() - start_time) < timeout:
#                 rospy.sleep(0.1)
#
#             if not self.callback:
#                 rospy.logwarn("No response received from NLP")
#                 self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
#                 return
#
#             self.callback = False
#
#             if self.response and self.response.get('intent') == 'Order':
#                 order_items = self.extract_order_from_response(self.response)
#                 print(order_items)
#                 if order_items:
#                     customer.set_order(order_items)
#                     self.confirm_order(customer, order_items)
#                 else:
#                     rospy.logwarn("No order was found in the response")
#             else:
#                 rospy.logwarn("Response was not an order")
#             tries += 1
#         if tries == 3:
#             TalkingMotion("Do you want to order a water?").perform()
