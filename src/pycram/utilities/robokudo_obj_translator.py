from pycram.ros.logging import logwarn

get_obj = {
    "Mueslibox": "breakfast_cereal.stl",
    "Metalmug": "jeroen_cup.stl",
    "Metalbowl": "bowl.stl",
    "Milkpack": "milk.stl",
    "Spoon": "spoon.stl",
}


def translate_obj(obj_type:str) -> str:
    try:
        result = get_obj[obj_type]
        return result
    except KeyError:
        logwarn(f"could bot find fitting object for {obj_type} in resources")
        return "box.urdf"
