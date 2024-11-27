import rospy
from pycram.utilities.robocup_utils import StartSignalWaiter, TextToSpeechPublisher

# Create an instance of the StartSignalWaiter
start_signal_waiter = StartSignalWaiter()
#load everfyhing world giskard robokudo....
# Wait for the start signal
talk = TextToSpeechPublisher()
talk.pub_now("heey")
start_signal_waiter.wait_for_startsignal()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")
