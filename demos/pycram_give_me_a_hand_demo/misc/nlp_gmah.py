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
class NLP_GMAH():
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

    def check_instructor(self, data):
        msgList = data[0]
        print(msgList)
        if msgList['intent'] == 'Instructor':
            return True

