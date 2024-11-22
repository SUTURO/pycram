import rospy

from pycram.designators.motion_designator import TalkingMotion
from pycram.process_module import real_robot
from pycram.utilities.robocup_utils import TextToSpeechPublisher
print(rospy.Time.now())

# Create an instance of the TextToSpeechPublisher
text_to_speech_publisher = TextToSpeechPublisher()
#text_to_speech_publisher.publish_text("start")
print("Hello, world!")
with real_robot:

    # signal start
    TalkingMotion("start").resolve().perform()
print(rospy.Time.now())

# Publish a text-to-speech request

