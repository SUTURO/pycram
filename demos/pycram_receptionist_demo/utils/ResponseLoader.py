import json
import random


class ResponseLoader:
    def __init__(self, json_file='convo_response.json'):
        self.json_file = json_file
        self.data = None

    def load_data(self):
        # JSON-Datei laden
        try:
            with open(self.json_file) as file:
                self.data = json.load(file)
                print("Data loaded successfully.")
        except FileNotFoundError:
            raise FileNotFoundError(f"JSON file {self.json_file} not found.")

    def predict_response(self, hobby_list):
        answered = False
        if self.data is None:
            raise ValueError("Data not loaded. Call `load_data()` first.")

        # test for hobby in json
        for hobby in hobby_list:
            if hobby in self.data['hobbies']:
                responses = self.data['hobbies'][hobby]
                answered = True
                return random.choice(responses)
                break
        if not answered:
            # unknown interest
            return random.choice(self.data['fallback'])
