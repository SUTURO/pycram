import json
import random


class LocationHelper:
    def __init__(self, json_file='location.json', ignore_words='ignore_words.json'):
        self.json_file = json_file
        self.ignore = ignore_words
        self.ignore_data = None
        self.data = None

    def load_file(self):
        """
        Method to manually load the JSON file where the NLP response is mapped to the current
        regions in our semantic map. This JSON file also includes the associated Poses (x,y,z)
        with those regions. Please feel free to extend the file for future expansions of the semantic
        map.
        """
        try:
            with open(
                    f"/home/suturo/suturo23_24/pycram_ws/src/pycram/demos/pycram_give_me_a_hand_demo/misc/{self.json_file}") as file:
                self.data = json.load(file)
                print("Successfully loaded Locations")

        except FileNotFoundError:
            raise FileNotFoundError(f"JSON file {self.json_file} not found")
        try:
            with open(f"/home/suturo/suturo23_24/pycram_ws/src/pycram/demos/pycram_give_me_a_hand_demo/misc/{self.ignore}") as file2:
                ignore = json.load(file2)
                self.ignore_data = ignore.get("ignore_words", [])
                print("Successfully loaded Ignore Data")
        except FileNotFoundError:
            raise FileNotFoundError(f"JSON file {self.ignore} not found")

    def is_valid_location_name(self, name: str) -> bool:
        """
        Method to filter unwanted str from the NLP response. Please feel free to expand on the json file for
        forbidden words, such as 'object', 'thing' etc.
        :param: name: str of the NLP response
        :return: if str is present in json file
        """
        return name.strip().lower() not in self.ignore_data
    def get_location(self, locName: str) -> str:
        """
        Method to find the official region name in the semantic map, given the understood location name.
        List of these possibilities is not finished yet.
        :param: locName: The NLP data
        :return: Either the current fallback location or the official perception name for the region
        """
        locName = locName.strip()
        locName = locName.lower()
        print("input name", locName)
        if self.data is None:
            raise ValueError("Please load the json first")

        if not locName in self.data['location']:
            return self.data['fallback']

        if locName in self.data['location']:
            try:
                resp = self.data['location'][locName]
                return random.choice(resp)
            except KeyError as e:
                print(f"Entry was not found due to {e}")

    def get_position(self, locName: str) -> (float, float, float):
        """
        Method to get the position of the official region in the semantic map from perception.
        Please make sure that you first got the official name of the region before trying to
        access the associated pose.
        :param: locName: The official name of the region
        :return: A tuple consisting of the x-, y- and z-position of the middle point of the region.
        """
        if self.data is None:
            raise ValueError("Please load the json first")
        print(self.data)
        print(type(self.data))

        if not locName in self.data['positions']:
            print("Hello")
            resp = self.data['fallback_position']
            x = resp[0].get('x')
            y = resp[0].get('y')
            z = resp[0].get('z')
            return x, y, z

        if locName in self.data['positions']:
            try:
                resp = self.data['positions'][locName]
                x = resp[0].get('x')
                y = resp[0].get('y')
                z = resp[0].get('z')
                return x,y,z
            except KeyError as e:
                print(f"Entry was not found due to {e}")

    def get_drive_positions(self, locName:str) -> (float, float, float):

        """
        Because we get the middle points of the tables as a result, we need to have specific drive poses for the robot
        """

        if self.data is None:
            raise ValueError("Please load the json first")

        if not locName in self.data['drive_positions']:
            resp = self.data['fallback_position']
            x = resp[0].get('x')
            y = resp[0].get('y')
            z = resp[0].get('z')
            return x, y, z

        if locName in self.data['drive_positions']:
            try:
                resp = self.data['drive_positions'][locName]
                x = resp[0].get('x')
                y = resp[0].get('y')
                z = resp[0].get('z')
                return x,y,z
            except KeyError as e:
                print(f"Entry was not found due to {e}")
