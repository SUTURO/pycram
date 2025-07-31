data = {'sentence': 'Bring the cup to the long table .',
        'intent': 'Transporting',
        'entities': [{'role': 'Item', 'value': 'cup', 'entity': 'Transportable',
                      'propertyAttribute': [], 'actionAttribute': [], 'numberAttribute': []},
                     {'role': 'Destination', 'value': 'long table', 'entity': 'DesignedFurniture',
                      'propertyAttribute': [], 'actionAttribute': [], 'numberAttribute': []}]}


def check_location(data):
    print(data)
    print(type(data))
    #msgList = data[0]
    #print(msgList)
    if data['intent'] == 'Transporting':
        print("Yipii")
        for  entity in data['entities']:
            if entity['role'] == 'Destination':
                loc = entity['value']
                return loc
    return None


check_location(data)