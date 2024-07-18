from enum import Enum

from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_serve_breakfast_demo.utils.misc import *
from pycram.utilities.robocup_utils import *
import pycram.external_interfaces.giskard_new as giskardpy
# from pycram.external_interfaces.knowrob import get_table_pose

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalbowl", "Cornflakes", "Milkpack", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the shelf
shelf_pose = 5.45

# x pose of the end of the shelf
tresse_pose = 10.15

# the bowl to pour in
bowl = None

# free places for placing
place_pose = None

# temp variable to save place pose
temp_place_pose = None

table_object_list = None

metalbowl_pose = None

# x pose for placing the object
y_pos = 5.25

# list of sorted placing poses
sorted_places = []

# An instance of the TextToSpeechPublisher
text_to_speech_publisher = TextToSpeechPublisher()

# An instance of the ImageSwitchPublisher
image_switch_publisher = ImageSwitchPublisher()

# An instance of the StartSignalWaiter
start_signal_waiter = StartSignalWaiter()

# An instance of the PoseNavigator
move = PoseNavigator()

# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
#TODO: change urdf
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")

giskardpy.init_giskard_interface()
giskardpy.clear()
giskardpy.sync_worlds()


class PlacingZPose(Enum):
    """
    Differentiate the z pose for placing
    """
    CUTLERY = 0.835
    SPOON = 0.835
    FORK = 0.835
    PLASTICKNIFE = 0.835
    KNIFE = 0.835
    METALBOWL = 0.875
    MILKPACK = 0.855  # old milk 0.745
    METALMUG = 0.835
    CORNFLAKES = 0.98
    METALPLATE = 0.935
    CRONYBOX = 0.86


def try_detect(pose: Pose, location: bool):
    """
    lets the robot looks on a pose and perceive objects or free spaces

    :param pose: the pose that the robot looks to
    :param location: if location should be detected or not
    :return: tupel of State and dictionary of found objects in the FOV
    """
    image_switch_publisher.pub_now(10)
    LookAtAction(targets=[pose]).resolve().perform()
    text_to_speech_publisher.pub_now("Perceiving")
    try:
        if location:
            object_desig = DetectAction(technique='location', state='popcorn_table').resolve().perform()
        else:
            object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    image_switch_publisher.pub_now(0)
    return object_desig


def navigate_to(x: float, y: float, table_name: str):
    """
    Navigates to popcorn table, long table or shelf.

    :param x: x pose to navigate to
    :param y: y pose to navigate to
    :param table_name: defines the name of the table to move to
    """
    if table_name == "front":
        move.pub_now(Pose([x, y, 0], [0, 0, 0, 1]))
    elif table_name == "left":
        move.pub_now(Pose([x, y, 0], [0, 0, 0.7, 0.7]))
    elif table_name == "right":
        move.pub_now(Pose([x, y, 0], [0, 0, -0.7, -0.7]))
    elif table_name == "behind":
        move.pub_now(Pose([x, y, 0], [0, 0, 1, 0]))

# def adjust_pose(lst):
#     global y_pos
#     for t_obj in lst:
#         t_obj_size = t_obj.dimensions.x
#         # max is the left side of the table
#         max_y = t_obj.pose.position.y + (t_obj_size / 2) + 0.1
#         min_y = t_obj.pose.position.y - (t_obj_size / 2) - 0.1
#         if min_y <= y_pos <= max_y:
#             while y_pos > min_y:
#                 y_pos -= 0.2

def adjust_pose(lst):
    global y_pos
    for t_obj in lst:
        # max is the left side of the table
        max_y = t_obj.pose.position.y + 0.125
        min_y = t_obj.pose.position.y - 0.125
        if min_y <= y_pos <= max_y:
            while y_pos > min_y:
                y_pos -= 0.2


def pickup_and_place_objects(sorted_obj: list, tresse: bool):
    """
    For picking up and placing the objects in the given object designator list.

    :param sorted_obj: the sorted list of seen object designators.
    """
    global shelf_pose, CUTLERY, bowl, place_pose

    for value in range(len(sorted_obj)):
        # define grasping pose
        grasp = "front"
        if sorted_obj[value].type in CUTLERY:
            sorted_obj[value].type = "Cutlery"

        if sorted_obj[value].type in ["Cornflakes"]:
            sorted_obj[value].type = "Cornflakes"

        if sorted_obj[value].type in ["Metalbowl", "Cutlery"]:
            grasp = "top"

        # TODO: muss noch getestet und angepasst werden
        if sorted_obj[value].type == "Cutlery":
            if tresse:
                # change object x pose if the grasping pose is too far in the table
                if sorted_obj[value].pose.position.x > tresse_pose + 0.125:
                    sorted_obj[value].pose.position.x -= 0.1
                sorted_obj[value].pose.position.z = 0.98
            else:
                # change object x pose if the grasping pose is too far in the table
                if sorted_obj[value].pose.position.x < shelf_pose - 0.125:
                    sorted_obj[value].pose.position.x += 0.1
                # sorted_obj[value].pose.position.x = 3.25

                if sorted_obj[value].pose.position.z >= 1:
                    sorted_obj[value].pose.position.z = 1.18
                elif sorted_obj[value].pose.position.z >= 0.65:
                    sorted_obj[value].pose.position.z = 0.733
                elif sorted_obj[value].pose.position.z >= 0.3:
                    sorted_obj[value].pose.position.z = 0.413
                else:
                    sorted_obj[value].pose.position.z = 0.097

        image_switch_publisher.pub_now(7)
        text_to_speech_publisher.pub_now("Picking up with: " + grasp)
        try_pick_up(robot, sorted_obj[value], grasp)
        image_switch_publisher.pub_now(0)

        # move back a little bit
        if tresse:
            navigate_to(robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y, "front")
        else:
            navigate_to(robot.get_pose().pose.position.x + 0.3, robot.get_pose().pose.position.y, "behind")

        place_objects(True, sorted_obj, value, grasp, tresse)


def place_objects(first_placing, objects_list, index, grasp, tresse):
    """
    places objects on the popcorn table

    :param first_placing: if the object has been picked up
    :param objects_list: list of given objects
    :param index: index to iterate in objects list
    :param grasp: define the way of grasping
    """
    global bowl, place_pose, sorted_places, y_pos, temp_place_pose, table_object_list, metalbowl_pose

    if first_placing:
        object_type = objects_list[index].type
    else:
        object_type = objects_list[index]

    # placing the object
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    text_to_speech_publisher.pub_now("Navigating")
    # MoveGripperMotion("open", "left").resolve().perform()

    if tresse:
        # turn around
        navigate_to(9, 6.2, "behind")
        # # from right
        # navigate_to(9.1, 2.85, "behind")
        # navigate_to(6.45, 2.5, "left")

        # from left
        navigate_to(6.35, 6.2, "right")
    else:
        # turn around
        navigate_to(6.45, 5.8, "right")
        if table_object_list is None:
            navigate_to(6.2, 4.55, "front")
            # third variant without regions
            MoveTorsoAction([0.3]).resolve().perform()
            table_obj_desig = try_detect(Pose([7.65, 4.7, 0.21], [0, 0, 0, 1]), False)
            table_object_list = get_objects(table_obj_desig, wished_sorted_obj_list)
            adjust_pose(table_object_list)
            metalbowl_pose = y_pos
    if object_type != "Metalbowl":
        navigate_to(6.9, metalbowl_pose, "front")
        object_desig = try_detect(Pose([7.65, metalbowl_pose, 0.21], [0, 0, 0, 1]), False)
        bowl = get_bowl(object_desig)

    # first variant
    ###############################################################################
    # if object_type != "Cutlery":
    #     navigate_to(2.1, 4.5, "front")
    #     place_poses_list = try_detect(Pose([7.75, 4.77, 0.21], [0, 0, 0, 1]), True)
    #     sorted_places = get_free_spaces(place_poses_list[1])
    #
    # place_pose = sorted_places[0]
    #
    # if object_type != "Metalbowl":
    #     navigate_to(6.65, place_pose.pose.position.y, "front")
    #     object_desig = try_detect(Pose([7.5, place_pose.pose.position.y, 0.21], [0, 0, 0, 1]), False)
    #     bowl = get_bowl(object_desig)

    # second variant without free places
    ###############################################################################
    # if object_type != "Metalbowl":
    #     navigate_to(1.6, 4.8, "popcorn table")
    #     object_desig = try_detect(Pose([1.6, 5.9, 0.21], [0, 0, 0.7, 0.7]), False)
    #     bowl = get_bowl(object_desig)
    # navigate_to(2, 4.8, "popcorn table")
    ###############################################################################

    if object_type in ["Cornflakes", "Milkpack", "Cutlery"]:
        if bowl is None:
            # move 30cm back
            navigate_to(robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y, "front")
            new_object_deign = try_detect(Pose([7.65, y_pos, 0.21], [0, 0, 0, 1]), False)
            #new_object_deign = try_detect(Pose([7.5, place_pose.pose.position.y, 0.21], [0, 0, 0, 1]), False)
            bowl = get_bowl(new_object_deign)

            if bowl is None:
                text_to_speech_publisher.pub_now(f"Can you please put the Metalbowl on the table?")
                rospy.sleep(5)
                final_object_deign = try_detect(Pose([7.65, y_pos, 0.21], [0, 0, 0, 1]), False)
                #final_object_deign = try_detect(Pose([7.5, place_pose.pose.position.y, 0.21], [0, 0, 0, 1]), False)
                bowl = get_bowl(final_object_deign)

                if bowl is None:
                    text_to_speech_publisher.pub_now(f"I can not find the Metalbowl."
                                                     "I will skip pouring and place the objects on the table")
                    if object_type == "Cutlery":
                        # TODO: nach Variante anpassen
                        y_pos += 0.2
                        # place_pose.pose.position.x += 0.2

        if bowl is not None:
            if object_type in ["Cornflakes", "Milkpack"]:
                # TODO: Werte anpassen
                navigate_to(6.9, bowl.pose.position.y + 0.1, "front")
                # print(f"arm_roll: {robot.get_joint_state('arm_roll_joint')}")
                angle = 115
                # TODO: add pouring image
                image_switch_publisher.pub_now(7)
                if robot.get_pose().pose.position.y < bowl.pose.position.y:
                    PouringAction([bowl.pose], ["left"], ["right"], [angle]).resolve().perform()
                else:
                    PouringAction([bowl.pose], ["left"], ["left"], [angle]).resolve().perform()
                    # Move away from the table
                    navigate_to(robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y,
                                "front")
                ParkArmsAction([Arms.LEFT]).resolve().perform()
            else:
                # TODO: je nach Variante x-pos oder place-pose anpassen
                temp_place_pose = y_pos
                y_pos = bowl.pose.position.y + 0.2
                # place_pose.pose.position.x = bowl.pose.position.x
                # place_pose.pose.position.y = bowl.pose.position.y + 0.2
                # place_pose.pose.position.z = bowl.pose.position.z

    # TODO: nach Variante anpassen
    navigate_to(6.9, y_pos, "front")
    # navigate_to(6.65, place_pose.pose.position.y, "popcorn table")
    # navigate_to(6.65, y_pos, "popcorn table")
    image_switch_publisher.pub_now(8)
    text_to_speech_publisher.pub_now("Placing")
    z = get_z(object_type)
    if first_placing:
        # TODO: place-pose.pose.position.x oder x-pos auswählen
        # PlaceAction(objects_list[index], ["left"], [grasp],
        #             [Pose([7.4, place_pose.pose.position.y, z])]).resolve().perform()
        PlaceAction(objects_list[index], ["left"], [grasp], [Pose([7.45, y_pos, z])]).resolve().perform()
    else:
        # TODO: place-pose.pose.position.x oder x-pos auswählen
        # PlaceGivenObjAction([objects_list[index]], ["left"],
        #                     [Pose([7.4, place_pose.pose.position.y, z])], [grasp]).resolve().perform()

        PlaceGivenObjAction([objects_list[index]], ["left"], [Pose([7.45, y_pos, z])],
                            [grasp]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()

    # for the second variant
    # if object_type == "Metalbowl":
    #     y_pos += 0.8
    # else:
    #     y_pos += 0.3

    y_pos -= 0.2

    if object_type == "Cutlery":
        y_pos = temp_place_pose

    # for third variant
    adjust_pose(table_object_list)

    image_switch_publisher.pub_now(0)
    # navigates back if a next object exists
    if index + 1 < len(objects_list):
        text_to_speech_publisher.pub_now("Navigating")
        if tresse:
            # # from right
            # navigate_to(robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y, "right")
            # navigate_to(6.45, 2.5, "right")
            # navigate_to(9.1, 2.85, "front")
            # navigate_to(9, 6.2, "left")
            # from left
            navigate_to(robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y, "left")
            navigate_to(6.35, 6.2, "left")
            navigate_to(9, 6.2, "front")

            if first_placing:
                navigate_to(9.8, sorted_obj[index + 1].pose.position.y, "front")
            else:
                navigate_to(9.55, 6, "front")
        else:
            if first_placing:
                navigate_to(6.9, sorted_obj[index + 1].pose.position.y, "behind")
            else:
                navigate_to(6.45, 5.8, "left")


def get_z(obj_type: str):
    """
    Getter for z value for placing the given object type.

    :param obj_type: type of object, which z pose we want
    :return: the int z value for placing that object
    """
    return PlacingZPose[obj_type.upper()].value


def remove_objects(value):
    """
    removes already transported objects from the list

    :param value: the object that should be removed from the list
    """
    # remove all objects that were seen and transported so far
    if value.type in wished_sorted_obj_list:
        wished_sorted_obj_list.remove(value.type)
    # if the type is cutlery, the real type is not easily reproducible.
    # remove one cutlery object of the list with the highest chance that it was found and transported.
    if value.type == "Cutlery":
        if "Spoon" in wished_sorted_obj_list:
            print("deleted fork")
            wished_sorted_obj_list.remove("Fork")
        elif "Fork" in wished_sorted_obj_list:
            print("deleted spoon")
            wished_sorted_obj_list.remove("Spoon")
        elif "Plasticknife" in wished_sorted_obj_list:
            print("deleted knife")
            wished_sorted_obj_list.remove("Plasticknife")


with real_robot:
    rospy.loginfo("Starting demo")
    # MoveGripperMotion("close", "left").resolve().perform()
    text_to_speech_publisher.pub_now("Starting demo")
    image_switch_publisher.pub_now(0)
    # # Wait for the start signal
    # start_signal_waiter.wait_for_startsignal()
    #
    # # Once the start signal is received, continue with the rest of the script
    # rospy.loginfo("Start signal received, now proceeding with tasks.")

    # TODO: vielleicht entfernen in der finalen Demo
    # MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    text_to_speech_publisher.pub_now("Navigating")
    # navigate from door to a place in front of shelf
    navigate_to(2.55, 0.5, "front")
    navigate_to(4.85, 0.55, "front")
    navigate_to(4.8, 3.6, "left")
    navigate_to(6.6, 3.7, "front")
    navigate_to(6.5, 5.8, "left")
    navigate_to(5.9, 5.81, "behind")

    # TODO: MoveTOrso anpassen oder entfernen
    MoveTorsoAction([0.2]).resolve().perform()
    obj_desig = try_detect(Pose([5.35, 5.8, 0.21], [0, 0, 1, 0]), False) # z = 0.45
    sorted_obj = sort_objects(obj_desig, wished_sorted_obj_list)
    text_to_speech_publisher.pub_now(f"I perceived")
    for i in range(len(sorted_obj)):
        text_to_speech_publisher.pub_now(f" a {sorted_obj[i].type}")
    print(sorted_obj[0].type)
    if sorted_obj[0].type != "Metalbowl":
        bowl_obj_desig = try_detect(Pose([5.35, 5.8, 0.21], [0, 0, 1, 0]), False) # z = 0.45
        sorted_obj = sort_objects(bowl_obj_desig, wished_sorted_obj_list)
        if sorted_obj[0].type != "Metalbowl":
            text_to_speech_publisher.pub_now(f"Can you please give me the Metalbowl")
            time.sleep(4)
            MoveGripperMotion("close", "left").resolve().perform()
            place_objects(False, ["Metalbowl"], 0, "top", False)
            # navigate back after placing bowl
            navigate_to(6.15, 5.87, "behind")
    pickup_and_place_objects(sorted_obj, False)

    # failure handling part 1
    new_sorted_obj = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive and pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far
            remove_objects(value)

        # navigate to tresse and perceive for objects
        text_to_speech_publisher.pub_now("Navigating")
        # # from right
        # navigate_to(6.45, 2.5, "right")
        # navigate_to(9.1, 2.85, "front")
        # navigate_to(9.4, 6, "left")
        # navigate_to(9.4, 6, "front")

        # from left
        navigate_to(6.35, 6.2, "left")
        navigate_to(9, 6.2, "front")
        navigate_to(9.4, 6, "front")

        MoveTorsoAction([0.4]).resolve().perform()
        new_object_desig = try_detect(Pose([10.4, 4.6, 0.21], [0, 0, 0, 1]), False)
        new_sorted_obj = sort_objects(new_object_desig, wished_sorted_obj_list)
        pickup_and_place_objects(new_sorted_obj, True)

        # failure handling part 2
        final_sorted_obj = sorted_obj + new_sorted_obj
        if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
            navigate_to(6.45, 5.8, "behind")
            print("second Check")

            for value in final_sorted_obj:
                # remove all objects that were seen and transported so far
                remove_objects(value)

            for val in range(len(wished_sorted_obj_list)):
                grasp = "front"
                if wished_sorted_obj_list[val] in (["Metalbowl"] + CUTLERY):
                    grasp = "top"

                image_switch_publisher.pub_now(5)
                print(f"next object is: {wished_sorted_obj_list[val]}")
                text_to_speech_publisher.pub_now(f"Can you please give me the {wished_sorted_obj_list[val]} "
                                                 f"I can not find it")
                time.sleep(4)
                MoveGripperMotion("close", "left").resolve().perform()

                image_switch_publisher.pub_now(0)

                place_objects(False, wished_sorted_obj_list, val, grasp, True)

    image_switch_publisher.pub_now(3)
    rospy.loginfo("Done!")
    text_to_speech_publisher.pub_now("Done")

########################### PSEUDO-CODE ################################
# if door open then enter room
# navigate to shelf
# perceive
# sort objects in a list like this (bowl , cereal, milk, spoon)
# loop: pickup object
####### navigate to table
####### if object milk or cereal or spoon -> perceive bowl
#######  if milk or cereal -> pour into bowl
#######   if spoon place object right of bowl, ELSE: place on free space
#######
