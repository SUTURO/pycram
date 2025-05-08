import re
import ast
from typing import List, Tuple, Dict, Any

from constants import OPTIONS, NUMBERS

class NLPProcessor:
    """Handles processing of NLP response and data formatting."""

    @staticmethod
    def get_order_data(nlp_response:str)->List[Tuple[str, int]]:
        """
        Extract order data from NLP response. Currently, works only with GPSR script.
        :param: nlp_response: Raw NLP response string
        :return: List of tuples (item, quantity)
        """
        order_list = []
        msg = ast.literal_eval(nlp_response)
        if msg['intent'] == "Order":
            list_order = msg['Item']
            tmp_entity = list_order['value']
            tmp_num = list_order['numberAttribute']

            list_entity = [tmp_entity]
            list_num = [1] if not tmp_num else [OPTIONS.get(tmp_num[0], 1)]

            order_list = list(zip(list_entity, list_num))

        return order_list

    @staticmethod
    def split_response(data: List[str]) -> List[str]:
        """
        Clean and split NLP response data.

        :param: data: List of strings to clean
        :return: Cleaned list of strings
        """
        new_tmp = [n.strip() for n in data]
        return [re.sub('\W+', '', m) for m in new_tmp]

    @staticmethod
    def split_number_word(input_data: List[Tuple[str, int]]) -> List[Tuple[str, int]]:
        """
        Split combined number-word strings into separate components.

        Args:
            input_data: List of (item, quantity) tuples

        Returns:
            Processed list with separated components
        """
        result = []

        for input_str in input_data:
            str_order = input_str[0]
            if input_str[1] != 1:
                result.append((input_str[0], input_str[1]))

            for number in NUMBERS:
                if str_order.startswith(number):
                    leftover = str_order[len(number):]
                    tmp_num = OPTIONS[number]
                    if (leftover, tmp_num) not in result:
                        result.append((leftover, tmp_num))

        return result

    @staticmethod
    def save_order(data: List[str]) -> List[Tuple[str, int]]:
        """
        Convert raw order data into structured format.

        Args:
            data: List of order items and quantities

        Returns:
            List of (item, quantity) tuples
        """
        tuple_order = [(x, OPTIONS[y]) for x, y in zip(data, data[1:]) if y in OPTIONS]

        for order in tuple_order:
            for num in NUMBERS:
                if num in order[0]:
                    return NLPProcessor.split_number_word(tuple_order)

        return tuple_order