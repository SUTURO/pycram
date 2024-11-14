from pycram.failures import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_serve_breakfast_demo.utils.misc import *
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.worlds.bullet_world import BulletWorld

# TODO: change postions of navigating, pickup, placing, etc.
# TODO: giskardpy not found
# TODO: DetectAction new parameter (object)

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalbowl", "Cerealbox", "Milkpackja", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the shelf
shelf_pose = 5.11

# the bowl to pour in
bowl = None

# free places for placing
place_pose = None

# x pose for placing the object
x_pos = 1.45

# list of sorted placing poses
sorted_places = []

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_12.urdf")


class PlacingZPose(Enum):
    """
    Differentiate the z pose for placing
    """
    CUTLERY = 0.775
    SPOON = 0.775
    FORK = 0.775
    PLASTICKNIFE = 0.775
    KNIFE = 0.775
    METALBOWL = 0.815
    MILKPACKJA = 0.845
    METALMUG = 0.775
    CEREALBOX = 0.875
    METALPLATE = 0.875
    CRONYBOX = 0.8


def try_detect(pose: Pose, location: bool):
    """
    lets the robot looks on a pose and perceive objects or free spaces

    :param pose: the pose that the robot looks to
    :param location: if location should be detected or not
    :return: tupel of State and dictionary of found objects in the FOV
    """
    LookAtAction(targets=[pose]).resolve().perform()
    TalkingMotion("Perceiving").perform()
    try:
        if location:
            object_desig = DetectAction(technique='location', state='popcorn_table').resolve().perform()
        else:
            object_desig = DetectAction(technique='all').resolve().perform()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def navigate_to(x: float, y: float, table_name: str):
    """
    Navigates to popcorn table, long table or shelf.

    :param x: x pose to navigate to
    :param y: y pose to navigate to
    :param table_name: defines the name of the table to move to
    """
    if table_name == "shelf":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 0, 1])]).resolve().perform()
    elif table_name == "popcorn table":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    elif table_name == "long table":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 1, 0])]).resolve().perform()


def pickup_and_place_objects(sorted_obj: list):
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

        if sorted_obj[value].type in ["Mueslibox", "Cerealbox", "Crackerbox"]:
            sorted_obj[value].type = "Cerealbox"

        if sorted_obj[value].type in ["Metalbowl", "Cutlery"]:
            grasp = "top"

        # TODO: muss noch getestet und angepasst werden
        if sorted_obj[value].type == "Cutlery":
            # change object x pose if the grasping pose is too far in the table
            if sorted_obj[value].pose.position.x > shelf_pose + 0.125:
                sorted_obj[value].pose.position.x -= 0.1
            # sorted_obj[value].pose.position.x = 3.25

            if sorted_obj[value].pose.position.z >= 0.65:
                sorted_obj[value].pose.position.z = 0.715
            elif sorted_obj[value].pose.position.z >= 0.3:
                sorted_obj[value].pose.position.z = 0.37
            else:
                sorted_obj[value].pose.position.z = 0.065

        TalkingMotion("Picking up with: " + grasp).perform()
        try_pick_up(robot, sorted_obj[value], grasp)

        # move back a little bit
        navigate_to(robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y, "shelf")

        place_objects(True, sorted_obj, value, grasp)


def place_objects(first_placing, objects_list, index, grasp):
    """
    places objects on the popcorn table

    :param first_placing: if the object has been picked up
    :param objects_list: list of given objects
    :param index: index to iterate in objects list
    :param grasp: define the way of grasping
    """
    global bowl, place_pose, sorted_places, x_pos

    if first_placing:
        object_type = objects_list[index].type
    else:
        object_type = objects_list[index]

    # placing the object
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    TalkingMotion("Navigating").perform()
    # MoveGripperMotion("open", "left").resolve().perform()
    navigate_to(3.6, 4.9, "long table")
    # navigate_to(2, 4.8, "popcorn table")

    # erste Variante
    ###############################################################################
    if object_type != "Cutlery":
        navigate_to(2.1, 4.5, "popcorn table")
        place_poses_list = try_detect(Pose([2.1, 5.9, 0.21], [0, 0, 0.7, 0.7]), True)
        sorted_places = get_free_spaces(place_poses_list[1])

    if object_type != "Metalbowl":
        navigate_to(1.6, 4.8, "popcorn table")
        object_desig = try_detect(Pose([1.6, 5.9, 0.21], [0, 0, 0.7, 0.7]), False)
        bowl = get_bowl(object_desig)

    place_pose = sorted_places[0]

    # zweite Variante ohne free places
    ###############################################################################
    # if object_type != "Metalbowl":
    #     navigate_to(1.6, 4.8, "popcorn table")
    #     object_desig = try_detect(Pose([1.6, 5.9, 0.21], [0, 0, 0.7, 0.7]), False)
    #     bowl = get_bowl(object_desig)
    # navigate_to(2, 4.8, "popcorn table")
    ##############################################################################

    if object_type in ["Cerealbox", "Cronybox", "Milkpackja", "Cutlery"]:
        if bowl is None:
            # move 30cm back
            navigate_to(robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.3, "popcorn table")
            new_object_deign = try_detect(Pose([1.6, 5.9, 0.21], [0, 0, 0.7, 0.7]), False)
            bowl = get_bowl(new_object_deign)

            if bowl is None:
                TalkingMotion(f"Can you please put the Metalbowl on the table?").perform()
                rospy.sleep(5)
                final_object_deign = try_detect(Pose([1.6, 5.9, 0.21], [0, 0, 0.7, 0.7]), False)
                bowl = get_bowl(final_object_deign)

                if bowl is None:
                    TalkingMotion(f"I can not find the Metalbowl.").perform()
                    TalkingMotion(f"I will skip pouring and place the objects on the table").perform()
                    if object_type is "Cutlery":
                        # TODO: nach Variante anpassen
                        # x_pos += 0.3
                        place_pose.pose.position.x += 0.3

        if bowl is not None:
            if object_type in ["Cerealbox", "Cronybox", "Milkpackja"]:
                # TODO: Werte anpassen
                navigate_to(bowl.pose.position.x, 5, "popcorn table")
                # print(f"arm_roll: {robot.get_joint_state('arm_roll_joint')}")
                angle = 115
                try:
                    if robot.get_pose().pose.position.x > bowl.pose.position.x:
                        direction = "right"
                    else:
                        direction = "left"
                    PouringAction([bowl.pose], [Arms.LEFT], [direction], [angle]).resolve().perform()
                except EnvironmentUnreachable:
                    navigate_to(robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.3,
                                "popcorn table")
                    ParkArmsAction([Arms.LEFT]).resolve().perform()
                    navigate_to(1.6, 4.8, "popcorn table")
                    object_desig = try_detect(Pose([1.6, 5.9, 0.21], [0, 0, 0.7, 0.7]), False)
                    bowl = get_bowl(object_desig)
                    if bowl is not None:
                        navigate_to(bowl.pose.position.x, 5, "popcorn table")
                        try:
                            if robot.get_pose().pose.position.x > bowl.pose.position.x:
                                direction = "right"
                            else:
                                direction = "left"
                            PouringAction([bowl.pose], [Arms.LEFT], [direction], [angle]).resolve().perform()
                        except EnvironmentUnreachable:
                            navigate_to(robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.3,
                                        "popcorn table")
                            ParkArmsAction([Arms.LEFT]).resolve().perform()
                            TalkingMotion(f"Pouring will be skipped").perform()
                            TalkingMotion(f"i can not reach the bowl on the table").perform()
                if bowl is not None:
                    # Move away from the table
                    navigate_to(robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.3,
                                "popcorn table")
                    ParkArmsAction([Arms.LEFT]).resolve().perform()
                else:
                    TalkingMotion(f"Pouring will be skipped").perform()
                    TalkingMotion(f"i can not find the bowl on the table").perform()
            else:
                # TODO: je nach Variante x-pos oder place-pose anpassen
                # x_pos = bowl.pose.position.x + 0.2
                place_pose.pose.position.x = bowl.pose.position.x + 0.2
                place_pose.pose.position.y = bowl.pose.position.y
                place_pose.pose.position.z = bowl.pose.position.z

    # TODO: nach Variante anpassen
    navigate_to(place_pose.pose.position.x, 5, "popcorn table")
    # navigate_to(x_pos, 5, "popcorn table")
    TalkingMotion("Placing").perform()
    z = get_z(object_type)
    if first_placing:
        # TODO: place-pose.pose.position.x oder x-pos auswählen
        PlaceAction(objects_list[index], [Arms.LEFT], [grasp],
                    [Pose([place_pose.pose.position.x, 5.8, z])]).resolve().perform()
        # PlaceAction(objects_list[index], ["left"], [grasp], [Pose([x_pos, 5.8, z])]).resolve().perform()
    else:
        # TODO: place-pose.pose.position.x oder x-pos auswählen
        PlaceGivenObjAction([objects_list[index]], [Arms.LEFT],
                            [Pose([place_pose.pose.position.x, 5.8, z])], [grasp]).resolve().perform()

        # PlaceGivenObjAction([objects_list[index]], ["left"], [Pose([x_pos, 5.8, z])],
        #                     [grasp]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()

    if object_type == "Metalbowl":
        x_pos += 0.8
    else:
        x_pos += 0.3

    # navigates back if a next object exists
    if index + 1 < len(objects_list):
        TalkingMotion("Navigating").perform()
        if first_placing:
            navigate_to(4.3, sorted_obj[index + 1].pose.position.y, "shelf")
        else:
            navigate_to(4.3, 4.9, "shelf")


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

    if value.type == "Cerealbox":
        if "Mueslibox" in wished_sorted_obj_list:
            print("deleted Mueslibox")
            wished_sorted_obj_list.remove("Mueslibox")
        elif "Cerealbox" in wished_sorted_obj_list:
            print("deleted Cerealbox")
            wished_sorted_obj_list.remove("Cerealbox")
        elif "Crackerbox" in wished_sorted_obj_list:
            print("deleted Crackerbox")
            wished_sorted_obj_list.remove("Crackerbox")
        elif "Cronybox" in wished_sorted_obj_list:
            print("deleted Cronybox")
            wished_sorted_obj_list.remove("Cronybox")


with (real_robot):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").perform()
    # TODO: vielleicht entfernen in der finalen Demo
    # MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    TalkingMotion("Navigating").perform()
    # navigate from door to a place in front of shelf
    # TODO: koordinaten bestimmen
    # navigate_to(2.1, 1.87, "shelf")
    navigate_to(2, 4.8, "popcorn table")

    # navigate to shelf
    navigate_to(4.3, 4.9, "shelf")
    try:
        MoveTorsoAction([0.2]).resolve().perform()
    except (TorsoLowLevelFailure, TorsoGoalNotReached):
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        try:
            MoveTorsoAction([0.2]).resolve().perform()
        except (TorsoLowLevelFailure, TorsoGoalNotReached):
            TalkingMotion(f"I can not move my Torso!").perform()
            TalkingMotion(f"I will try to perceive without moving it!").perform()
    obj_desig = try_detect(Pose([5.25, 4.9, 0.21], [0, 0, 0, 1]), False)
    sorted_obj = sort_objects(obj_desig, wished_sorted_obj_list)
    print(sorted_obj[0].type)
    if sorted_obj[0].type != "Metalbowl":
        # navigate to shelf
        navigate_to(4.3, 4.9, "shelf")
        bowl_obj_desig = try_detect(Pose([5.25, 4.9, 0.21], [0, 0, 0, 1]), False)
        sorted_obj = sort_objects(bowl_obj_desig, wished_sorted_obj_list)
        if sorted_obj[0].type != "Metalbowl":
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            TalkingMotion(f"Can you please give me the Metalbowl in the shelf?").perform()
            rospy.sleep(4)
            MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
            place_objects(False, ["Metalbowl"], 0, "top")
            navigate_to(4.3, 4.9, "shelf")
    pickup_and_place_objects(sorted_obj)

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

        TalkingMotion("Navigating").perform()
        navigate_to(4.3, 4.9, "shelf")
        new_object_desig = try_detect(Pose([5.25, 4.9, 0.21], [0, 0, 0, 1]), False)
        new_sorted_obj = sort_objects(new_object_desig, wished_sorted_obj_list)
        pickup_and_place_objects(new_sorted_obj)

        # failure handling part 2
        final_sorted_obj = sorted_obj + new_sorted_obj
        if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
            navigate_to(4.3, 4.9, "shelf")
            print("second Check")

            for value in final_sorted_obj:
                # remove all objects that were seen and transported so far
                remove_objects(value)

            for val in range(len(wished_sorted_obj_list)):
                grasp = "front"
                if wished_sorted_obj_list[val] in (["Metalbowl"] + CUTLERY):
                    grasp = "top"

                print(f"next object is: {wished_sorted_obj_list[val]}")
                TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} "
                              f"in the shelf?").perform()
                rospy.sleep(4)
                MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()

                place_objects(False, wished_sorted_obj_list, val, grasp)

    rospy.loginfo("Done!")
    TalkingMotion("Done").perform()