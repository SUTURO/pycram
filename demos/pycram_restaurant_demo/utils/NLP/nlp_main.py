import rospy
from std_msgs.msg import String

from demos.pycram_restaurant_demo.utils.NLP.order_manager import OrderManager
from demos.pycram_restaurant_demo.utils.NLP.speech_manager import SpeechManager
from pycram.datastructures.enums import ImageEnum
from pycram.designators.motion_designator import HeadFollowMotion


class RestaurantNLP:
    """Main class for restaurant NLP interaction."""

    def __init__(self):
        self.speech_manager = SpeechManager()
        self.order_manager = OrderManager(self.speech_manager)
        self.sub_nlp = rospy.Subscriber("nlp_out", String, self.speech_manager.data_cb)

    def data_cb(self, data):
        """Callback for NLP data."""
        self.speech_manager.image_switch_publisher.pub_now(ImageEnum.HI.value)
        self.speech_manager.test = data.data
        self.speech_manager.response = data.data.split(",")
        self.speech_manager.response.append("None")
        self.speech_manager.callback = True

    def order_ready(self) -> bool:
        """Check if order is ready."""
        HeadFollowMotion(state='start').perform()
        return self.speech_manager.confirm_action(
            "Please confirm that the order is ready with a yes after my display changes.",
            "<CONFIRM>"
        )

    def took_order(self) -> bool:
        """Check if customer took their order."""
        HeadFollowMotion(state='start').perform()
        self.speech_manager.say_multiple([
            "Here is your order.",
            "Please say yes if you took your order after my display changes."
        ])
        rospy.sleep(2.5)
        return self.speech_manager.confirm_action("", "<CONFIRM>")