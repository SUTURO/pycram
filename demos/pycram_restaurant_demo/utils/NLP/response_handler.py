import time
from typing import List

import rospy

from pycram.datastructures.enums import ImageEnum
from pycram.designators.motion_designator import TalkingMotion
from pycram.utilities.robocup_utils import ImageSwitchPublisher, TextToImagePublisher


class ResponseHandler:
    """Handles all interaction with the user"""
    def __init__(self):
        self.image_publisher = ImageSwitchPublisher()
        self.text_publisher = TextToImagePublisher()

    def confirm_order(self, order: List[tuple]) -> bool:
        """
        Confirmation of received order by the customer.
        :param: order: The order
        :return: If order was correct
        """
        self._display_confirmation(order)
        response = self._get_user_response()
        return self._interpret_response()

    def _display_confirmation(self, order: List[tuple]):
        """Creates text image from order
        :param: order: The order"""
        if len(order) == 1:
            item, quantity = order[0]
            TalkingMotion(f"Do you want to order {quantity} {item}?").perform()
            self.text_publisher.pub_now(f"Order: {quantity} {item}")
        else:
            TalkingMotion("Do you want to order: ").perform()
            for item, quantity in order:
                TalkingMotion(f"{quantity} {item}").perform()
                rospy.sleep(1)
            self.text_publisher.pub_now(", ".join([f"{q} {i}" for i, q in order]))

    def _get_user_response(self, timeout: int = 10) -> dict:
        self.nlp_pub.publish("start")
        self.image_publisher.pub_now(ImageEnum.JREPEAT.value)

        start_time_rep = time.time()
        while not self.callback:
            rospy.sleep(1)
            if int(time.time() - start_time_rep) == timeout:
                rospy.logwarn("guest needs to repeat")
                self.image_publisher.pub_now(ImageEnum.JREPEAT.value)
                rospy.sleep(2)

    def _interpret_response(self, response: dict) -> bool:
        """Interpret user response
        :param: response: The response of the user """
        if isinstance(response, dict):
            return response.get('intent') == "Confirm"
