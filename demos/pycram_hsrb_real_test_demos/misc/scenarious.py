import rospy

from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
sound_publisher = SoundRequestPublisher()

text_to_speech_publisher.pub_now("Hey! I am Toya.")
image_switch_publisher.pub_now(15)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("Talk to me.")
image_switch_publisher.pub_now(1)
#sound_publisher.publish_sound_request()
# rospy.sleep(2)
text_to_speech_publisher.pub_now("Opening the dishwasher.")
image_switch_publisher.pub_now(2)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("I dropped the object.")
image_switch_publisher.pub_now(4)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("Please Handover the object")
image_switch_publisher.pub_now(5)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("What will you odering?")
image_switch_publisher.pub_now(6)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("I am picking up the object.")
image_switch_publisher.pub_now(7)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("I am placing the object.")
image_switch_publisher.pub_now(8)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("Can you repeat that?")
image_switch_publisher.pub_now(9)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("I am searching for the object18263821638126381263812638126 nananan hehehehehhehe whaduwgudgau wadhk.")
text_to_speech_publisher.pub_now("I am searching for the object.")
image_switch_publisher.pub_now(10)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("Wave your hand.Waiting...")
image_switch_publisher.pub_now(11)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("Following human")
image_switch_publisher.pub_now(12)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("driving back")
image_switch_publisher.pub_now(13)
# rospy.sleep(2)
text_to_speech_publisher.pub_now("push my buttons tadadaa")
image_switch_publisher.pub_now(14)
# rospy.sleep(4)
text_to_speech_publisher.pub_now("follow and stop")
image_switch_publisher.pub_now(15)
# rospy.sleep(4)

# text_to_speech_publisher.publish_text("I am done.")
# image_switch_publisher.publish_image_switch(3)

# Hints: List for image view (mit Zahlen ändert man das Bild)
# "hi.png" -> 0
# "talk.png" -> 1
# "dish.png" -> 2
# "done.png" -> 3
# "drop.png" -> 4
# "handover.png" -> 5
# "order.png" -> 6
# "picking.png" -> 7
# "placing.png" -> 8
# "repeat.png" -> 9
# "search.png" -> 10
# "waving.mp4" -> 11
# "following" -> 12
# "drivingback" -> 13
# "pushbuttons" -> 14

