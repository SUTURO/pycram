import unittest

import json
class MiscTests(unittest.TestCase):

    def get_objects(self):
        data = []
        data = json.dumps({
"sentence": "  Bring me the yellow object from the table .",
  "intent": "Transporting", "BeneficiaryRole": {"value": "me", "entity": "NaturalPerson",
                    "propertyAttribute": [], "actionAttribute": [], "numberAttribute": []},
  "Item": {"value": "cup", "entity": "Transportable", "propertyAttribute": ["yellow"], "actionAttribute": [], "numberAttribute": []},
  "Source": {"value": "table", "entity": "DesignedFurniture", "propertyAttribute": [],
                               "actionAttribute": [], "numberAttribute": []}
}
)

        response = json.loads(data)
        tmp = response['Item']
        tmp_color = tmp['propertyAttribute']
        tmp_obj = tmp['value']

        return (tmp_obj, tmp_color[0])

    def test_get_desirec_input(self):
        print(self.get_objects())