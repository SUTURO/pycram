from enum import Enum
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_hsrb_real_test_demos.utils.misc import *
# from pycram.external_interfaces.knowrob import get_table_pose

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Bowl", "Metalmug", "Fork", "Cerealbox"]


# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# y pose for placing the object
y_pos = 1.66

# x pose of the end of the popcorn table
table_pose = 1.04

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchenmarch1.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)


class PlacingZPose(Enum):
    """
    Differentiate the z pose for placing
    """
    CUTLERY = 0.8
    SPOON = 0.8
    FORK = 0.8
    PLASTICKNIFE = 0.8
    KNIFE = 0.8
    BOWL = 0.84
    MILKPACK = 0.88
    METALMUG = 0.8
    CEREALBOX = 0.9
    METALPLATE = 0.9  # table height plus radius of plate


def pickup_and_place_objects(sorted_obj: list):
    """
    For picking up and placing the objects in the given object designator list.

    :param sorted_obj: the distance sorted list of seen object designators.
    """
    global y_pos, table_pose, CUTLERY

    for value in range(len(sorted_obj)):
        print("first navigation")

        # define grasping pose
        grasp = "front"
        if sorted_obj[value].type in CUTLERY:
            sorted_obj[value].type = "Cutlery"

        if sorted_obj[value].type in ["Bowl", "Cutlery"]:
            grasp = "top"

        if sorted_obj[value].type == "Metalplate":
            TalkingMotion("Can you please give me the plate on the table.").resolve().perform()
            MoveGripperMotion("open", "left").resolve().perform()
            time.sleep(3)
            MoveGripperMotion("close", "left").resolve().perform()
        else:
            # change object x pose if the grasping pose is too far in the table
            if sorted_obj[value].type == "Cutlery" and sorted_obj[value].pose.position.x < table_pose - 0.125:
                sorted_obj[value].pose.position.x += 0.1

            TalkingMotion("Picking up with: " + grasp).resolve().perform()
            try_pick_up(robot, sorted_obj[value], grasp)

        # placing the object
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        navigate_to(1.8, 1.8, "long table")
        navigate_to(4.1, y_pos, "long table")
        TalkingMotion("Placing").resolve().perform()

        z = get_z(sorted_obj[value].type)
        if sorted_obj[value].type == "Metalplate":
            # with special defined placing movement for the plate
            PlaceGivenObjAction([sorted_obj[value].type], ["left"],
                                [Pose([4.86, y_pos, z])],["front"]).resolve().perform()
        else:
            PlaceAction(sorted_obj[value], ["left"], [grasp],
                        [Pose([4.87, y_pos, z])]).resolve().perform()

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        navigate_to(3.9, 2, "popcorn table")

        # adjust y_pos for the next placing round
        if sorted_obj[value].type == "Metalplate":
            y_pos += 0.3
        else:
            y_pos += 0.16

        # navigates back if a next object exists
        if value + 1 < len(sorted_obj):
            navigate_to(1.6, sorted_obj[value + 1].pose.position.y, "popcorn table")


def get_z(obj_type: str):
    """
    Getter for z value for placing the given object type.

    :param obj_type: type of object, which z pose we want
    :return: the int z value for placing that object
    """
    return PlacingZPose[obj_type.upper()].value


def navigate_to(x: float, y: float, table_name: str):
    """
    Navigates to the popcorn table or to the table on the other side.

    :param x: x pose to navigate to
    :param y: y pose to navigate to
    :param table_name: defines the name of the table to move to
    """
    if table_name == "popcorn table":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 1, 0])]).resolve().perform()
    elif table_name == "long table":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 0, 1])]).resolve().perform()


def navigate_and_detect():
    """
    Navigates to the popcorn table and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    TalkingMotion("Navigating").resolve().perform()
    # navigate_to(False, 1.8, "popcorn table")
    navigate_to(1.7, 1.8, "popcorn table")  # 1.6

    # popcorn table
    LookAtAction(targets=[Pose([0.8, 1.8, 0.21], object_orientation)]).resolve().perform()  # 0.18 vanessa
    TalkingMotion("Perceiving").resolve().perform()
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


# Main interaction sequence with real robot
with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()
    MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    # detect objects
    object_desig = navigate_and_detect()

    # sort objects based on distance and which we like to keep
    sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)

    # picking up and placing objects
    pickup_and_place_objects(sorted_obj)

    # failure handling part 1
    new_sorted_obj = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive and pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            if value.type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.type)

        new_object_desig = navigate_and_detect()
        new_sorted_obj = sort_objects(robot, new_object_desig, wished_sorted_obj_list)
        pickup_and_place_objects(new_sorted_obj)

        # failure handling part 2
        final_sorted_obj = sorted_obj + new_sorted_obj
        if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
            navigate_to(1.6, 1.8, "popcorn table")
            print("second Check")

            for value in final_sorted_obj:
                # remove all objects that were seen and transported so far
                if value.type in wished_sorted_obj_list:
                    wished_sorted_obj_list.remove(value.type)
                # if the type is cutlery, the real type is not easily reproducible.
                # remove one cutlery object of the list with the highest chance that it was found and transported.
                if value.type == "Cutlery":
                    if "Fork" in wished_sorted_obj_list:
                        print("deleted fork")
                        wished_sorted_obj_list.remove("Fork")
                    elif "Spoon" in wished_sorted_obj_list:
                        print("deleted spoon")
                        wished_sorted_obj_list.remove("Spoon")
                    elif "Plasticknife" in wished_sorted_obj_list:
                        print("deleted knife")
                        wished_sorted_obj_list.remove("Plasticknife")

            for val in range(len(wished_sorted_obj_list)):
                grasp = "front"
                if wished_sorted_obj_list[val] in (["Bowl"] + CUTLERY):
                    grasp = "top"

                print(f"next object is: {wished_sorted_obj_list[val]}")
                TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} "
                              f"on the table?").resolve().perform()
                time.sleep(4)
                MoveGripperMotion("close", "left").resolve().perform()

                # if the Metalplate was first not found and is now placed somewhere in the
                # middle of the table, add space to the right object.
                if wished_sorted_obj_list[val] == "Metalplate":
                    y_pos += 0.14

                z = get_z(wished_sorted_obj_list[val])

                ParkArmsAction([Arms.LEFT]).resolve().perform()
                TalkingMotion("Navigating").resolve().perform()
                # navigate_to(True, 1.8, "long table")
                # navigate_to(False, y_pos, "long table")
                navigate_to(1.8, 1.8, "long table")
                navigate_to(4.1, y_pos, "long table")
                TalkingMotion("Placing").resolve().perform()

                PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"], [Pose([4.86, y_pos, z])],
                                    [grasp]).resolve().perform()
                ParkArmsAction([Arms.LEFT]).resolve().perform()

                if wished_sorted_obj_list[val] == "Metalplate":
                    y_pos += 0.3
                else:
                    y_pos += 0.16

                # navigates back if a next object exists
                # todo sollen wir ihn echt zum tisch navigieren lassen, wenn er das objekt vom menschen bekommt?
                if val + 1 < len(wished_sorted_obj_list):
                    TalkingMotion("Navigating").resolve().perform()
                    # navigate_to(True, 2, "popcorn table")
                    # navigate_to(False, 1.8, "popcorn table")
                    navigate_to(3.9, 2, "popcorn table")
                    navigate_to(1.6, 1.8, "popcorn table")

    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()


# def navigate_to(turn_around, y, table_name):
#     """
#     Navigates to the popcorntable or to the table on the other side.
#
#     :param x: x pose to navigate to
#     :param y: y pose to navigate to
#     :param orientation: defines the orientation of the robot respectively the name of the table to move to
#     """
# table = get_table_pose(table_name)

#       print(f"table_pose: {table}")
# if turn_around:
#  NavigateAction(target_locations=[
#                   Pose([2.0, y,
#                         table.posestamped.pose.position.z], [table.posestamped.pose.orientation.x,
#                         table.posestamped.pose.orientation.y, table.posestamped.pose.orientation.z,
#                                                             table.posestamped.pose.orientation.w])]).resolve().perform()
# else:
#     NavigateAction(target_locations=[table.posestamped.pose.position.x, y,
#      #                         table.posestamped.pose.position.z], [table.posestamped.pose.orientation.x,
#      #                         table.posestamped.pose.orientation.y, table.posestamped.pose.orientation.z,
#      #                                                             table.posestamped.pose.orientation.w])]).resolve().perform()]).resolve().perform()
