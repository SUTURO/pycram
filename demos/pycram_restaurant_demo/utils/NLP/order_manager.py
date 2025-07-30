from typing import List, Tuple
import rospy
import time

from demos.pycram_restaurant_demo.utils.NLP.nlp_processor import NLPProcessor
from demos.pycram_restaurant_demo.utils.NLP.speech_manager import SpeechManager
from pycram.datastructures.enums import ImageEnum
from pycram.designators.motion_designator import HeadFollowMotion
from pycram.designators.object_designator import CustomerDescription


class OrderManager:
    """Handles order taking and confirmation workflows."""

    def __init__(self, speech_manager: SpeechManager):
        self.speech = speech_manager
        self.nlp_processor = NLPProcessor()

    def get_order(self, customer: CustomerDescription) -> bool:
        """
        Main order taking workflow.

        Args:
            customer: Customer to take order from

        Returns:
            True if order successfully taken
        """
        HeadFollowMotion(state='start').perform()

        # Initial prompt
        self.speech.say_multiple([
            "Welcome, what can I get for you?",
            "Please come close to me and order when my display changes"
        ])

        rospy.sleep(2.5)
        self.speech.start_listening()

        if not self.speech.wait_for_response():
            return False

        if self.speech.response[0] == "<ORDER>":
            tmp = self.nlp_processor.split_response(self.speech.response)
            order = self.nlp_processor.save_order(tmp)
            if order:
                customer.set_order(order)
                return True

        return False

    def confirm_order(self, customer: CustomerDescription, order: List[Tuple[str, int]]) -> bool:
        """
        Order confirmation workflow.

        Args:
            customer: Customer to confirm with
            order: Order to confirm

        Returns:
            True if order confirmed
        """
        HeadFollowMotion(state='start').perform()

        if len(order) == 1:
            item_text = f"Do you want to order {order[0][1]} {order[0][0]}?"
            return self._confirm_single_item(item_text, customer)
        else:
            return self._confirm_multiple_items(order, customer)

    def _confirm_single_item(self, prompt: str, customer: CustomerDescription) -> bool:
        """Handle confirmation for single item orders."""
        self.speech.text_to_image_publisher.pub_now(prompt.replace("Do you want to order ", ""))
        self.speech.say_multiple([
            prompt,
            "Please confirm with a yes or no after my display changes"
        ])

        rospy.sleep(2.5)
        self.speech.image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
        self.speech.start_listening()

        if not self.speech.wait_for_response():
            return False

        if self.speech.response[0] == "<CONFIRM>":
            HeadFollowMotion(state='stop').perform()
            return True
        elif self.speech.response[0] == "<DENY>":
            return self.repeat_get_order(customer)

        return False

    def _confirm_multiple_items(self, order: List[Tuple[str, int]], customer: CustomerDescription) -> bool:
        """Handle confirmation for multiple item orders."""
        txt_order = " ".join([f"{n[1]} {n[0]}" for n in order])
        self.speech.text_to_image_publisher.pub_now(txt_order)

        self.speech.say_multiple([
            "Do you want to order the following items",
            *[f"{n[1]} {n[0]} and" for n in order],
            "Confirm your order with a yes, after my display changes"
        ])

        rospy.sleep(2.5)
        self.speech.image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
        self.speech.start_listening()

        if not self.speech.wait_for_response():
            return False

        if self.speech.response[0] == "<CONFIRM>":
            HeadFollowMotion(state='stop').perform()
            return True
        elif self.speech.response[0] == "<DENY>":
            return self.repeat_get_order(customer)

        return False

    def repeat_get_order(self, customer: CustomerDescription) -> bool:
        """Handle order repetition workflow."""
        self.speech.say_multiple([
            "Please repeat your order when my display changes"
        ])

        rospy.sleep(2.3)
        self.speech.start_listening()

        if not self.speech.wait_for_response():
            return False

        if self.speech.response[0] == "<ORDER>":
            tmp = self.nlp_processor.split_response(self.speech.response)
            order = self.nlp_processor.save_order(tmp)
            if order:
                customer.set_order(order)
                return self.confirm_order(customer, order)

        return False

    def give_order(self, order: List[Tuple[str, int]]):
        """Present order to bar staff."""
        HeadFollowMotion(state='start').perform()
        rospy.sleep(2)

        if len(order) == 1:
            self.speech.say(f"Please prepare the order {order[0][1]} {order[0][0]}")
        else:
            self.speech.say_multiple([
                "Please prepare the following order",
                *[f"{n[1]}{n[0]}" for n in order]
            ])