import ast
import json

msg = "[{'sentence': 'I would like to order two coffees , a steak and cola .', 'intent': 'Order', 'entities': {0: {'idx': 0, 'role': 'Item', 'value': 'coffees', 'group': 0, 'entity': 'drink', 'propertyAttribute': (), 'actionAttribute': (), 'numberAttribute': ('two',)}, 1: {'idx': 1, 'role': 'Item', 'value': 'steak', 'group': 0, 'entity': 'food', 'propertyAttribute': (), 'actionAttribute': (), 'numberAttribute': ()}, 2: {'idx': 2, 'role': 'Item', 'value': 'cola', 'group': 0, 'entity': 'drink', 'propertyAttribute': (), 'actionAttribute': (), 'numberAttribute': ()}}}]"

data = ast.literal_eval(msg)

json_str = json.dumps(data)

print(data)
print(type(data))

print(data[0]["intent"])
print(data[0]["entities"])

list_order = data[0]["entities"]
list_entity = []
list_num = []

options = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
           '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10, }

numbers = {'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten', 'eleven', 'twelve', '1', '2',
           '3',
           '4', '5', '6', '7', '8', '9', '10'}
for values in list_order.values():
    print(values)
    print(values["value"])
    list_entity.append(values["value"])
    tmp_num = values["numberAttribute"]
    if tmp_num == ():
        list_num.append(1)
    else:
        tmp = tmp_num[0]
        real_int = options.get(tmp)
        list_num.append(real_int)

order_list = list(zip(list_entity, list_num))

print(order_list)
