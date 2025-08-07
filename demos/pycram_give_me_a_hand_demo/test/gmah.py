from demos.pycram_give_me_a_hand_demo.misc.location_helper import LocationHelper

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

def check_json():
    data = ['white table', 'dining table', 'kitchen counter']
    test_fallback = 'kitchen table'
    locHelp = LocationHelper("location.json")
    locHelp.load_file()
    for n in data:
        tmp2 = locHelp._get_location(n)

        tmp = locHelp._get_position(tmp2)
        print(n)
        print("position", tmp)
        print(type(tmp))
        print("location", tmp2)
    tmpTest2 = locHelp._get_location(test_fallback)

    tmpTest = locHelp._get_position(test_fallback)
    print(tmpTest)
    print(tmpTest2)

check_json()


check_location(data)