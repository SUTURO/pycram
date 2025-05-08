from typing import Dict, Set

# Global constants
TIMEOUT = 10
RESPONSE_TEMPLATE = [None, None]
CONFIRMATION_TEMPLATE = [None]

# Number Mapping
OPTIONS: Dict[str, int] = {
    'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
    'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
    '1': 1, '2': 2, '3': 3, '4': 4, '5': 5,
    '6': 6, '7': 7, '8': 8, '9': 9, '10': 10
}

NUMBERS: Set[str] = {
    'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten',
    'eleven', 'twelve', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10'
}