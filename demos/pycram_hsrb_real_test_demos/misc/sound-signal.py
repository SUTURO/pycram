# Create an instance of the SoundRequestPublisher
from pycram.local_transformer import LocalTransformer
from pycram.utilities.robocup_utils import SoundRequestPublisher
from sensor_msgs.msg import LaserScan
from sound_play.msg import SoundRequestActionGoal, SoundRequest
from tmc_msgs.msg import Voice
from pycram.designators.object_designator import *

sound_publisher = SoundRequestPublisher()

# Publish a sound request
sound_publisher.publish_sound_request()
