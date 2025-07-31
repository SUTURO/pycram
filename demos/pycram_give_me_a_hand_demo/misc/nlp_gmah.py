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

# "{"sentence": "Bring the cup to the brown table .",
# "intent": "Transporting"\
#   , "entities": [{"role": "Item", "value": "cup", "entity": "Transportable"\
#   , "propertyAttribute": [], "actionAttribute": [], "numberAttribute": []},\
#   \ {"role": "Destination", "value": "table", "entity": "DesignedFurniture"\
#   , "propertyAttribute": ["brown"], "actionAttribute": [], "numberAttribute"\
#   : []}]}"

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

    def check_instructor(self):

        TalkingMotion("I could not see the desired location.").perform()
        rospy.sleep(2)
        TalkingMotion("Please tell me the location after my display changes").perform()
        rospy.sleep(1)

        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)
        msgList = self.response[0]
        print(msgList)
        if msgList['intent'] == 'Callout':
            return True


    def check_location(self):
        HeadFollowMotion(state='start').perform()
        TalkingMotion("I could not see the desired location.").perform()
        rospy.sleep(2)
        TalkingMotion("Please tell me the location after my display changes").perform()
        rospy.sleep(1)


        self.nlp_pub.publish("start listening")
        rospy.sleep(2.3)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

        print(self.response)
        print(type(self.response))
        #msgList = data[0]
        #print(msgList)

        start_time = time.time()
        while not self.callback and (time.time() - start_time) < timeout:
            rospy.sleep(0.1)
        if not self.callback:
            rospy.logwarn("No response received from NLP")
            self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
        self.callback = False


        if self.response['intent'] == 'Transporting':
            print("Yipii")
            for  entity in self.response['entities']:
                if entity['role'] == 'Destination':
                    loc = entity['value']
                    return loc
        return None
