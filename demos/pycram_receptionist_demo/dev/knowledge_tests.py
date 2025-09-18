import rospy

from neem_interface_python import rosprolog_client

ros_client = rosprolog_client.Prolog()


##### Infos about Knowrob querys #####
# communication via strings
# query once (one result) und query all solutions (all results)
# dot at the end of query string important, if . is not there query won't stop
# there is a difference between "" and '', you will need 'data'

#### Launching Knowledge ############
# type in new terminal
# kn (sources knowledge workspace)
# mongodb     (um Datenbank zu aktivieren)
# roslaunch suturo_knowledge suturo_knowledge.launch
# (rosrun suturo_knowledge object_info_server.py) old

#######################################


def save_person_drink(name: str, drink: str):
    """
    save person and their favorite drink to knowledge base
    :param name: name of person being saved
    :param drink: favorite drink of that person
    """
    x = False
    name.lower()
    if "coffee" in drink.lower():
        x = ros_client.once("save_me_and_coffee('" + name + "').")
    if "raspberryjuice" in drink.lower():
        x = ros_client.once("save_me_and_raspberryjuice('" + name + "').")
    if "water" in drink.lower():
        x = ros_client.once("save_me_and_water('" + name + "').")
    if "tea" in drink.lower():
        x = ros_client.once("save_me_and_tea('" + name + "').")
    if "milk" in drink.lower():
        x = ros_client.once("save_me_and_milk('" + name + "').")

    if x:
        print(f"saved name: {name} and drink: {drink}")
        rospy.loginfo("saved name: {name} and drink: {drink}")
    else:
        rospy.logwarn("could not save name and drink")


def known_person(name: str):
    """
    checks if a person is saved in knowledge base
    :param name: name of person
    """
    name = name.lower()
    query = "is_customer('" + name + "')."
    known = ros_client.once(query)
    if known:
        print(f"we know {name}")
        return True
    else:
        print(f"we do not know {name}")
        return False


def get_fav_drink(name: str):
    """
    returns favorite drink of a person
    checks if person is known if person is unknown or has no favorite drink,
    False is returned
    :param name: name of person whose favorite drink is returned
    """
    if known_person(name):
        name = name.lower()
        query = "fav_drink('" + name + "'," + "X)."
        data = ros_client.once(query)
        print(data)

        uri = data['X']
        name_with_suffix = uri.split('#')[-1]
        drink = name_with_suffix.split('_')[0]
        print(f"{name} drinks {drink}")
        return drink
    else:
        return False


def get_pose(room: str):
    """
    beta version of get pose in room function
    query not correct yet and result needs to be changed
    """
    if "kitchen" in room.lower():
        data = ros_client.once("has_type(Room, 'http://www.ease-crc.org/ont/SOMA.owl#Kitchen'), entry_pose(Room, EntryPose), "
                        "exit_pose(Room, ExitPose).")
        return data


# tests functions like this:
save_person_drink("bob", "coffee")
get_fav_drink("bob")



