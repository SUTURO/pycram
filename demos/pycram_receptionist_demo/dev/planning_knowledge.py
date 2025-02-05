#!/usr/bin/env python3

import rospy
# import rosprolog_client
from geometry_msgs.msg import PoseStamped
from neem_interface_python import rosprolog_client

# ros_client = rosprolog_client.Prolog()

prolog = rosprolog_client.Prolog()


class PrologInterface():

    def __init__(self) -> None:
        self.PlanningKnowledge = InterfacePlanningKnowledge()


# prol = PrologInterface()

# status = prol.PlanningKnowledge.save_person_and_drink(info)

#########################################################################
# 1:
# Save name X and favourite drink Y

class InterfacePlanningKnowledge:

    def save_person_and_drink(self, name, drink):
        name_part = name.lower()

        print("1.Name:", name_part)
        print("2.Drink:", drink)
        # print("3.ID:", id_part)

        # check if a fav drink already exists
        query_check = "fav_drink(" + name_part + ", X)."
        check_for_drink = prolog.once(query_check)
        print("query" + str(check_for_drink))

        if check_for_drink == []:

            # id = id_part

            if drink == "Coffee":
                # query = "save_me_and_coffee(" + name_part + "," + "\'" + id + "\')."
                query = "save_me_and_coffee(" + name_part + ")."

                rospy.loginfo(query)
                prolog.once(query)

            elif drink == "RaspberryJuice":
                # query = "save_me_and_raspberryjuice(" + name_part  + "," + "\'" + id + "\')."
                query = "save_me_and_raspberryjuice(" + name_part + ")."

                rospy.loginfo(query)
                prolog.once(query)

            elif drink == "Milk":
                # query = "save_me_and_milk(" + name_part  + "," + "\'" + id + "\')."
                query = "save_me_and_milk(" + name_part + ")."

                rospy.loginfo(query)
                prolog.once(query)

            elif drink == "Tea":
                # query = "save_me_and_tea(" + name_part  + "," + "\'" + id + "\')."
                query = "save_me_and_tea(" + name_part + ")."

                rospy.loginfo(query)
                prolog.once(query)

            elif drink == "Water":
                # query = "save_me_and_water(" + name_part  + "," + "\'" + id + "\')."
                query = "save_me_and_water(" + name_part + ")."

                rospy.loginfo(query)
                prolog.once(query)

            else:
                return ("Sorry " + name_part.capitalize() + ", but we don't know a drink named: "
                        + drink)

            return ("Your name is " + name_part.capitalize()
                    + " and your favourite drink is " + drink
                    + "." + " We saved your information!"
                    )
        else:
            print(drink)
            rospy.loginfo("We already registered your favourite drink")
            return ("We already registered your favourite drink as " + crop_plus(str(check_for_drink)) + ".")

    #########################################################################
    # 2:
    # Is person X already known to us?

    def do_we_know_u(self, name):
        # crop the input string to a useful string
        crop_string = name.lower()
        # count = 1
        query = "is_customer(" + crop_string + ")."
        solution = prolog.once(query)

        # save only when name is not already known
        if solution == dict():
            rospy.loginfo("Welcome back " + crop_string.capitalize() + "!")
            return True

        else:
            # count += 1
            # save = "save_me("+ crop_string + "," + str(count) + ")."
            save = "save_me(" + crop_string + ")."

            prolog.once(save)
            rospy.loginfo("We saved you!")
            rospy.loginfo("Nice to meet you " + crop_string.capitalize() + "!")

            # test if saving was successful
            test = "is_customer(" + crop_string + ")."
            rospy.loginfo("Test:" + str(test))
            test_call = prolog.once(test)

            if test_call != dict():
                rospy.loginfo("test_call: not successful")

            else:
                rospy.loginfo("test_call: successful")

            # because person was not known previously
            return False

    #########################################################################
    # 3:
    # What's person X's favourite drink?

    def what_is_your_fav_drink(self, name):
        # crop the input string to a useful string
        crop_string = name.lower()
        query = "fav_drink(" + crop_string + "," + "X)."
        solution = prolog.once(query)
        sol = crop(solution)

        give_type = "has_type(" + str(sol) + "," + "X)."
        ref = prolog.once(give_type)
        soll = crop_plus(crop(ref))
        return soll.replace("\'", "")

    #########################################################################
    # 4:
    # Where should this object be placed in the shelve?
    # Vielleicht noch das Object am Ende erstellen, wo das Object hingestellt werden soll.

    def challenge_storing_groceries(self):
        # init_storing_groceries auslagern, soll nur einmal aufgerufen werden
        q0 = "init_storing_groceries"
        prolog.once(q0)
        print(q0)

    def place_pose_object(self, object):
        print(object)

        # Input: apple (soll platziert werden)
        # knowledge erstellt random ein object apple
        # dann wir place pose bestimmt
        # Output: place pose vom apple
        #
        # ODER
        #
        # Input: pose vom apple ist bekannt (durch perception), so dass create_object aufgerufen wird
        # mit dem erstellten object, dann object_destination_pose aufrufen
        # Output: place pose vom apple

        already_exists = "create_object(Object, suturo:" + "\'" + "MetalBowl" + "\'" + ", [" + "\'" + "iai_kitchen/shelf:shelf:shelf_floor_2" + "\'" + ", [0.049,0.02,0], [0,0,0,1.0]], [shape(box(0.5,0.5,0.5))])."
        print(already_exists)
        solq = prolog.once(already_exists)
        print(solq)

        q3 = "what_object(" + "\'" + str(object) + "\'" + ",X)."
        print(q3)
        de_object = prolog.once(q3)
        print(de_object)

        place_it = "create_object(Object," + str(crop(
            de_object)) + ", [" + "\'" + "map" + "\'" + ", [0.049,0.02,0], [0,0,0,1.0]], [shape(box(0.5,0.5,0.5))])."
        solqq = prolog.once(place_it)

        print(solqq)

        sol = crop(solqq)
        print(sol)
        print("wait for a bit pls")
        q1 = "object_destination_pose(" + str(sol) + ",[], X)."
        solution = prolog.once(q1)
        print(solution)

        return solution

    #########################################################################
    # 5:
    # Get the table pose

    def get_pose(self, table_name):
        # get all known tables
        print(table_name)
        q1 = "has_type(X, soma:'Table')."
        sol = prolog.all_solutions(q1)

        # for all tables, ask for their poses
        if len(sol) != 0:
            for table in list(sol):
                print("tables:" + str(table))
                new_table = crop2(table).replace('}', "")
                q2 = "object_pose(" + str(new_table) + ",X)."
                tpos = prolog.once(q2)

                # extract table name and table frame
                table_frame = str(list(tpos.items())[0][1][0])
                print("1:" + str(table_name))
                print("2:" + str(table_frame))

                # verwende has_robocup_name in furniture_info.pl
                # compare whether there is a table named "table_name" and has searched frame
                if table_name == "p_table" and table_frame == "iai_kitchen/popcorn_table:p_table:table_center":
                    return build_posestamped(new_table)

                elif table_name == "l_table" and table_frame == "iai_kitchen/long_table:l_table:table_center":
                    return build_posestamped(new_table)

                else:
                    print("No right table")

        else:
            print("No solution tables")
            return None

        ###########################################################################

    # 6:
    # Get the handle pose
    # Wie verwendet das Planning? Was genau wollen die haben?
    # Posestamped zum Handlenamen | Frame zum Handlenamen

    def get_pose_of_handle(self, handle_name):
        q1 = "has_type(X, soma:'DesignedHandle')."
        sol = prolog.all_solutions(q1)

        # for all handles, ask for their pose
        if len(sol) != 0:
            for handle in list(sol):
                print("handles:" + str(handle))
                new_handle = crop2(handle).replace('}', "")
                print("print new_handle:" + new_handle)

                q2 = "object_pose(" + str(new_handle) + ",X)."
                print(q2)
                tpos = prolog.once(q2)
                handle_frame = str(list(tpos.items())[0][1][0])
                print("1:" + str(handle_name))
                print("2:" + str(handle_frame))

                # verwende has_robocup_name in furniture_info.pl
                # compare whether there is a handle named "handle_name" and has searched frame
                if handle_name == "k_door_inside" and handle_frame == "iai_kitchen/iai_kitchen:arena:door_handle_inside":
                    return build_posestamped(new_handle)
                elif handle_name == "k_door_outside" and handle_frame == "iai_kitchen/iai_kitchen:arena:door_handle_outside":
                    return build_posestamped(new_handle)
                elif handle_name == "l_door_inside" and handle_frame == "iai_kitchen/living_room:arena:door_handle_inside":
                    return build_posestamped(new_handle)
                elif handle_name == "l_door_outside" and handle_frame == "iai_kitchen/living_room:arena:door_handle_outside":
                    return build_posestamped(new_handle)
                elif handle_name == "dishwasher" and handle_frame == "iai_kitchen/sink_area_dish_washer_door_handle":
                    return build_posestamped(new_handle)
                elif handle_name == "shelf_left" and handle_frame == "iai_kitchen/shelf:shelf:shelf_door_left:handle":
                    return build_posestamped(new_handle)
                elif handle_name == "shelf_right" and handle_frame == "iai_kitchen/shelf:shelf:shelf_door_right:handle":
                    return build_posestamped(new_handle)
                else:
                    print("No right handle")

        else:
            print("No solution handles")
            return None

        #################################################################################

    # 7:
    # Check if an object is fragile
    def fragility_check(self, name):
        q1 = "what_object(" + "\'" + name.lower() + "\'" + ", Object)."
        sol = prolog.once(q1)

        if len(sol) == 0:
            print("Sorry, object is not known to us!")
            return False

        else:
            q2 = "fragility_new(" + "\'" + name.lower() + "\')."
            soll = prolog.once(q2)

            if soll == dict():
                print("Object is fragile!")
                return True

            else:
                print("Object exists but not fragile!")
                return False

    #################################################################################
    # 8:
    # Create an object
    def create_object(self, objname, pose):
        if len(pose) == 0:
            pose = ['map', [0, 0, 0], [0, 0, 0, 1]]
        q1 = "what_object(" + "\'" + objname.lower() + "\'" + ", Object)."
        sol = prolog.once(q1)
        print(sol)
        newname = crop(sol)
        print(newname)
        print(pose)

        q2 = "create_object(X," + newname + ", " + str(pose) + "). "
        sol2 = prolog.once(q2)
        print(sol2)
        return sol2

    #################################################################################
    # 9:
    # get the pose from where an object shall be grabbed

    def grasp_pose(self, objname):
        q1 = "grasp_pose(" + "\'" + objname.lower() + "\'" + ", Pose)."
        sol = prolog.once(q1)
        newname = crop(sol)
        print(newname)
        return newname

    #################################################################################
    # 10:
    # get the pose where an object shall be placed

    def place_destination(self, objname):
        q1 = "has_position(" + "\'" + objname.lower() + "\'" + ", Pose)."
        print(q1)
        sol = prolog.once(q1)
        print(sol)
        return sol


#################################################################################

## Get the object pose that depends on certain object property
# just a draft
"""
    def where_at(name):
        obj_name = crop2(name)
        q1 = "what_object(" + "\'" + obj_name.lower() + "\'" + ", Object)."
        sol = prolog.once(q1)

        if len(sol) != 0:
            q2 = "is_perishable("+ "\'" + obj_name + "\')."
            soll = prolog.once(q2)

            if soll == dict():
                return get_pose("furniture:popcorn table")

            else: 
                print("Object exists but not perishable. You can find it on the kitchen counter!")
                return get_pose("furniture:kitchen counter")

        else:
            print("No object found under this name!")
            #return get_pose("furniture:long table")
            return None
"""


###########################################################################
## return information about object as PoseStamped
# just for formating

def build_posestamped(new_table):
    new_pose = "object_pose(" + str(new_table) + ", [map, X, Y])."
    sol = prolog.once(new_pose)
    print("4:" + str(sol))

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position.x = sol['X'][0]
    pose_stamped.pose.position.y = sol['X'][1]
    pose_stamped.pose.position.z = sol['X'][2]

    pose_stamped.pose.orientation.x = sol['Y'][0]
    pose_stamped.pose.orientation.y = sol['Y'][1]
    pose_stamped.pose.orientation.z = sol['Y'][2]
    pose_stamped.pose.orientation.w = sol['Y'][3]

    return pose_stamped


#########################################################################
# crop magic

# crop unnecessary chars and begin after the ':'
def crop(string):
    dpunkt_index = str(string).find(":")
    if dpunkt_index != -1:
        ex_string = str(string)[dpunkt_index + 1:]
        # print("ex_string:" + ex_string)
        de_string = ex_string.strip(' "').rstrip('}')
        # print("de_string:" + de_string)

        return de_string


# crop magic
def crop2(name):
    dpunkt_index = str(name).find(":")
    if dpunkt_index != -1:
        ex_string = str(name)[dpunkt_index + 1:]
        de_string = ex_string.strip(' "')
        return de_string


# crop everything before the '#'
def crop_plus(string):
    # search '#'
    hi = str(string).find('#')
    if hi != -1:
        new_str = str(string)[hi + 1:]
        return new_str
    else:
        print("No '#'")



testobj = InterfacePlanningKnowledge()
testobj.save_person_and_drink("Peter", "Tea")
rospy.sleep(2)
testobj.do_we_know_u("Peter")
print("dooooone")
