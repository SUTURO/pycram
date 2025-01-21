import json
import random


class ResponseLoader:
    def __init__(self, json_file='resp.json'):
        self.json_file = json_file
        self.data = None

    def load_data(self):
        # JSON-Datei laden
        try:
            with open("/home/suturo/suturo23_24/pycram_ws/src/pycram/demos/pycram_receptionist_demo/utils/resp.json") as file:
                self.data = json.load(file)
                print("Data loaded successfully.")
        except FileNotFoundError:
            print(self.json_file)
            raise FileNotFoundError(f"JSON file {self.json_file} not found.")

    def predict_response(self, hobby_list):
        answered = False
        if self.data is None:
            raise ValueError("Data not loaded. Call `load_data()` first.")

        # test for hobby in json
        # print(eval(hobby_list))
        if not hobby_list:
            return random.choice(self.data['fallback'])

        for hobby in hobby_list:
            if hobby in self.data['hobbies']:

                responses = self.data['hobbies'][hobby]
                answered = True
                return random.choice(responses)
                break
        if not answered:
            print("in fallback!")
            # unknown interest
            return random.choice(self.data['fallback'])
