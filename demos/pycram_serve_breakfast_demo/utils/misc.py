import math

from pycram.world_concepts.world_object import Object


def sort_objects(robot: Object, obj_dict: dict, wished_sorted_obj_list: list):
    """
    Transforms the given object dictionary to a distance sorted list.
    The Metalplate, if seen, is arranged as the last object in the list.

    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :param wished_sorted_obj_list: list of object types we like to keep
    :param robot: the robot
    :return: distance sorted list of seen and wished to keep objects
    """
    sorted_objects = []
    object_dict = {}
    containsPlate = False
    metalplate = None

    if len(obj_dict) == 0:
        return sorted_objects

    # cut of the given State and keep the dictionary
    # first, *remaining = obj_dict
    robot_pose = robot.get_pose()
    # calculate euclidian distance for all found object in a dictionary
    #for dictionary in remaining:
    for value in obj_dict.values():
        distance = math.sqrt(pow((value.pose.position.x - robot_pose.pose.position.x), 2) +
                             pow((value.pose.position.y - robot_pose.pose.position.y), 2) +
                             pow((value.pose.position.z - robot_pose.pose.position.z), 2))

        # fill dict with objects and their distance.
        # Add seen objects only once, if seen multiple times
        if value.obj_type not in obj_dict:
            object_dict[value.obj_type] = (value, distance)

    # sort all objects that where in the obj_dict
    sorted_object_list = sorted(object_dict.items(), key=lambda distance: distance[1][1])
    sorted_object_dict = dict(sorted_object_list)

    # keep only objects that are in the wished list and put Metalplate last
    for (object, distance) in sorted_object_dict.values():
        if object.obj_type in wished_sorted_obj_list:
            if object.obj_type == "Metalplate":
                containsPlate = True
                metalplate = object
            else:
                sorted_objects.append(object)

    # when all objects are sorted in the list add the plate at last
    if containsPlate:
        # or making Metalplate to the first object in list: sorted_objects.insert(0, metalplate)
        sorted_objects.append(metalplate)

    # print which objects are in the final list
    test_list = []
    for test_object in sorted_objects:
        test_list.append(test_object.obj_type)
    print(test_list)

    return sorted_objects