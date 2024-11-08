import unittest

import json
class MiscTests(unittest.TestCase):

    def get_objects(self):
        data = []
        data = dict({'sentence': 'Please bring me the red cup .',
'intent': 'Transporting', 'object-name': '',
'object-type': '', 'person-name': 'me', 'person-type': '', 'object-attribute': '', 'person-action': '', 'color': '', 'number': '', 'from-location': '', 'to-location': '', 'from-room': '', 'to-room': ''}

)

        response = json.loads(data)
        tmp = response['sentence']
        resp = tmp.split(" ")
        print(resp)


        return resp

    def test_get_desirec_input(self):
        res = self.get_objects()
        print(res[4])
        print(res[5])
        print(self.get_objects())