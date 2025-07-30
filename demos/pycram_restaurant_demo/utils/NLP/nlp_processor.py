import json
import re
import ast
from typing import List, Tuple, Dict, Any

from demos.pycram_restaurant_demo.utils.NLP.constants import OPTIONS, NUMBERS


class NLPProcessor:
    """Handles processing of NLP response and data formatting."""
    def __init__(self):
        self.options = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
           '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10,}

        self.numbers = {'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten', 'eleven', 'twelve', '1', '2', '3',
           '4', '5', '6', '7', '8', '9', '10'}

    def parse_input(self, data: str) -> dict:
        """
        Parses input data, handling both JSON and legacy string formats
        """
        try:
            return json.loads(data)
        except json.JSONDecodeError:
            print("Error, msg is not a json")

    def extract_order(self, parsed_data: dict) -> List[tuple]:
        """Extract order item from parsed data.
        :param: parsed_data: JSON data from NLP
        :return: List of tuples from the order"""

        if not parsed_data.get('intent') == "Order":
            return []
        return [
            (entity['value'], self._get_quantity(entity))
            for entity in parsed_data.get('entities', {}).values()
        ]
    def _get_quantity(self, entity: dict) -> int:
        """Exchanges the written number of the entity data to the int version.
        :param: entity: The entity from which we want to know the quantity of
        :return: the int version of the quantity"""
        num_attr = entity.get('numberAttribute', ())
        return self.options.get(num_attr[0], 1) if num_attr else 1

