import ast
import json
import time
from typing import List, Dict, Any
import rospy
from std_msgs.msg import String

from demos.pycram_give_me_a_hand_demo.misc.location_helper import LocationHelper
from pycram.datastructures.enums import ImageEnum
from pycram.designators.motion_designator import HeadFollowMotion, TalkingMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.utilities.robocup_utils import ImageSwitchPublisher, TextToImagePublisher

response = []
confirmation = []
callback = False
timeout = 15



text_to_image_pub = TextToImagePublisher()
image_switch_pub = ImageSwitchPublisher()



class NLP_GMAH():
    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self._data_callback)
        self.response = None
        self.location_helper = LocationHelper("location.json", "ignore_words.json")
        self.location_helper.load_file()
        self.callback = False
        self.image_switch_publisher = ImageSwitchPublisher()
        self.text_to_image_pub = TextToImagePublisher()

    def parse_nlp_response(self, data: str):
        print(data)
        try:
            return json.loads(data)
        except:
            rospy.logwarn("Failed to parse NLP")
            return None

    def _data_callback(self, data):
        """
        Receives the data from NLP and dumps it into a JSON, to optimize working with it.

        """
        self.parse_json_string(data.data)
        self.callback = True
    ##### EXAMPL SENTENCE ####
    # {"sentence": "Please bring the object to the kitchen counter .",
    # "intent": "Transporting",
    # "entities":
    # [{"role": "Item", "value": "object", "entity": "Transportable", "propertyAttribute": [], "actionAttribute": [], "numberAttribute": []},
    # {"role": "Destination", "value": "kitchen counter", "entity": "DesignedFurniture", "propertyAttribute": [], "actionAttribute": [], "numberAttribute": []}]}

    def parse_json_string(self, json_string: str):
        """
        Method to transfrom the received data from NLP
        """
        global destination, item
        print(json_string)
        try:
            parsed = json.loads(json_string)
            intent = parsed.get('intent')
            print(intent)
            if intent == "Transporting":
                entities = parsed.get('entities')
                for entity in entities:
                    item = ""
                    destination = ""
                    if entity.get('role') == "Item":
                        item = entity.get('value')
                    elif entity.get('role') == "Destination":
                        destination = entity.get('value')
                    print(item)
                    print(destination)
                is_invalid = self.location_helper.is_valid_location_name(item)
                print(is_invalid)
                if not is_invalid:
                    location = item + " " + destination
                else:
                    location = destination
                self.response = [intent, location]
        except (ValueError, SyntaxError, IndexError) as e:
            print(f"Error parsing string: {e}")
            self.response = ["Transporting", "long table"]

    def find_location(self, timeout=15, max_tries=3):
        """
        Method that is called if Perception returns no known location.
        Guides the user to repeat where the object should go using NLP
        """
        self._guide_user_to_speak()
        rospy.sleep(2)
        for attempt in range(max_tries + 1):
            self._start_listening()
            success, official_name, pose, drive_pose = self._handle_nlp_response()

            if success:
                HeadFollowMotion(state='stop').perform()
                return official_name, pose, drive_pose
            else:
                rospy.logwarn("Did not understand the location, asking to repeat")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
        rospy.logwarn("Failed to get a valid location after multiple attempts")
        return None, None

    def _guide_user_to_speak(self):
        """
        Helper method to make the HSR say the needed phrases
        """
        HeadFollowMotion(state='start').perform()
        rospy.sleep(2)

        phrases = [
            "Sorry, I could not make out where I should put this object",
            "Please come close to me and tell me where I should put it",
            "Please use the sentence: Please bring the object to the table after my display changes"
        ]
        for phrase in phrases:
            TalkingMotion(phrase).perform()
            rospy.sleep(2.8)

    def _start_listening(self):
        """
        Helper Method to start the NLP side of this challenge
        """
        print("NLP start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

    def _handle_nlp_response(self, timeout=15):
        """
        Helper Method to handle firstly the first NLP response as well as the possibility
        that the HSR did not understand the user correctly
        """
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if time.time() - start_time > timeout:
                return False, None, None

        self.callback = False

        if self.response[0] == "Transporting":
            loc = self.response[1]
            if loc:
                offical_name = self.location_helper.get_location(loc)
                pose = self.location_helper.get_position(offical_name)
                drive_pose = self.location_helper.get_drive_positions(offical_name)
                if pose:
                    return True, offical_name, pose, drive_pose
        return False, None, None

