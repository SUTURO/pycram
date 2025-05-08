import rospy
from std_msgs.msg import String
import time

from pycram.datastructures.enums import ImageEnum
from pycram.designators.motion_designator import TalkingMotion, HeadFollowMotion
from pycram.utilities.robocup_utils import ImageSwitchPublisher, TextToImagePublisher
from constants import TIMEOUT


class SpeechManager:
    """Manages speech interactions and NLP communication."""

    def __init__(self):
        self.nlp_pub = rospy.Publisher('/startListener', String, queue_size=16)
        self.image_switch_publisher = ImageSwitchPublisher()
        self.text_to_image_publisher = TextToImagePublisher()
        self.callback = False
        self.response = ["", ""]

    def wait_for_response(self) -> bool:
        """
        Wait for NLP response with timeout handling.

        Returns:
            True if response received, False if timed out
        """
        start_time = time.time()
        while not self.callback:
            rospy.sleep(1)
            if int(time.time()) - start_time == TIMEOUT:
                rospy.logwarn("Guest needs to repeat")
                self.image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
                return False
        self.callback = False
        return True

    def start_listening(self):
        """Initiate listening mode with visual feedback."""
        rospy.loginfo("nlp start")
        self.nlp_pub.publish("start listening")
        rospy.sleep(2)
        self.image_switch_publisher.pub_now(ImageEnum.TALK.value)

    def confirm_action(self, prompt: str, success_response: str) -> bool:
        """
        Generic confirmation dialog flow.

        Args:
            prompt: What to say to the user
            success_response: Expected positive response

        Returns:
            True if confirmed, False otherwise
        """
        TalkingMotion(prompt).perform()
        rospy.sleep(2)
        self.start_listening()

        if not self.wait_for_response():
            return False

        return self.response[0] == success_response