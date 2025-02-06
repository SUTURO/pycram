from typing_extensions import Optional

from demos.pycram_clean_the_table_demo.utils.misc import get_objects
from pycram.failures import *
from pycram.language import Code
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_serve_breakfast_demo.utils.misc import *
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

# TODO: change postions of navigating, pickup, placing, etc.
# TODO: giskardpy not found
# TODO: DetectAction new parameter (object)

new_objects = ["RedBullCane", "MilkpackLactoseFree", "FreezerBags", "MuesliboxVitalis", "TeeBagBoxWestminster"]

fts = ForceTorqueSensor(robot_name='hsrb')

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalbowl", "Cerealbox", "Milkpack", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the shelf
shelf_pose = 5.36

# the bowl to pour in
bowl = None

# free places for placing
place_pose = None

# x pose for placing the object
x_pos = 4.9

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

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")


class NavigatePose(Enum):
    DOOR = Pose([-1.58, 0, 0], [0, 0, 0, 1])
    CORRIDOR = Pose([3.32, 1.04, 0], [0, 0, 0.7, 0.7])
    KITCHEN_TABLE = Pose([4.49, 5.22, 0], [0, 0, -0.7, 0.7])  # +- 0.2 for left/ right on x
    BEFORE_KITCHEN = Pose([1.67, 6.15, 0], [0, 0, 0.7, 0.7])
    IN_KITCHEN = Pose([1.27, 8.66, 0], [0, 0, 0, 1])
    DISHWASHER_CLOSED = Pose([4.55, 8.75, 0], [0, 0, -0.7, 0.7])
    DISHWASHER_RIGHT = Pose([4.55, 8.75, 0], [0, 0, 0, 1])
    # DISHWASHER_LEFT = Pose([4.55, 8.75, 0], [0, 0, 1, 0])
    SHELF = Pose([4.62, 5.95, 0], [0, 0, 0, 1])


def try_detect(pose: Pose, technique: Optional[str] = None):
    """
    lets the robot looks on a pose and perceive objects or free spaces
    :param pose: the pose that the robot looks to
    :param technique: if location should be detected or not
    :return: tupel of State and dictionary of found objects in the FOV
    """
    LookAtAction(targets=[pose]).resolve().perform()
    TalkingMotion("Perceiving").perform()
    try:
        if technique == "location":
            object_desig = DetectAction(technique='location', state='popcorn_table').resolve().perform()
        else:
            object_desig = DetectAction(technique='all').resolve().perform()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def pickup_and_place_objects(sorted_obj: list):
    """
    For picking up and placing the objects in the given object designator list.
    :param sorted_obj: the sorted list of seen object designators.
    """
    global shelf_pose, CUTLERY, bowl, place_pose

    for value in range(len(sorted_obj)):
        # define grasping pose
        grasp = Grasp.FRONT
        if sorted_obj[value].obj_type in ["Mueslibox", "Cerealbox", "Crackerbox"]:
            sorted_obj[value].obj_type = "Cerealbox"
        if sorted_obj[value].obj_type in CUTLERY or sorted_obj[value].obj_type == "Metalbowl":
            grasp = Grasp.TOP
        # TODO: muss noch getestet und angepasst werden
        if sorted_obj[value].obj_type in CUTLERY:
            # change object x pose if the grasping pose is too far in the table
            if sorted_obj[value].pose.position.x > shelf_pose + 0.125:
                sorted_obj[value].pose.position.x -= 0.1
            # sorted_obj[value].pose.position.x = 3.25
            if sorted_obj[value].pose.position.z >= 0.9:
                sorted_obj[value].pose.position.z = 1.07
            elif sorted_obj[value].pose.position.z >= 0.4:
                sorted_obj[value].pose.position.z = 0.54
            else:
                sorted_obj[value].pose.position.z = 0.09
        TalkingMotion("Picking up from: " + str(grasp)[6:]).perform()
        try_pick_up(robot, sorted_obj[value], grasp)
        # move back a little bit
        NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x - 0.5,
                                               robot.get_pose().pose.position.y, 0],
                                              NavigatePose.SHELF.value.pose.orientation)]).resolve().perform()
        place_objects(True, sorted_obj, value, grasp)


def place_objects(first_placing: bool, objects_list: list, index: int, grasp: Grasp):
    """
    places objects on the popcorn table
    :param first_placing: if the object has been picked up
    :param objects_list: list of given objects
    :param index: index to iterate in objects list
    :param grasp: define the way of grasping
    """
    global bowl, place_pose, sorted_places, x_pos
    if first_placing:
        object_type = objects_list[index].obj_type
    else:
        object_type = objects_list[index]
    # placing the object
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveTorsoAction([0]).resolve().perform()

    NavigateAction(NavigatePose.KITCHEN_TABLE.value).resolve().perform()
    if object_type != "Metalbowl":
        NavigateAction([Pose([NavigatePose.KITCHEN_TABLE.value.pose.position.x + 0.2,
                              NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                             NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
        object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.21],
                                       NavigatePose.KITCHEN_TABLE.value.pose.orientation))
        bowl = get_bowl(object_desig)
    ##############################################################################
    if object_type in ["Cerealbox", "Cronybox", "Milkpack", "MilkpackLactoseFree", "MuesliboxVitalis"] or \
            object_type in CUTLERY:
        if bowl is None:
            # move 30cm back
            NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                                   robot.get_pose().pose.position.y + 0.3, 0],
                                                  NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
            MoveTorsoAction([0.12]).resolve().perform()
            new_object_deign = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.35],
                                               NavigatePose.KITCHEN_TABLE.value.pose.orientation))
            bowl = get_bowl(new_object_deign)
            if bowl is None:
                TalkingMotion(f"Can you please put the Metalbowl on the table?").perform()
                rospy.sleep(5)
                final_object_deign = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.35],
                                                     NavigatePose.KITCHEN_TABLE.value.pose.orientation))
                bowl = get_bowl(final_object_deign)
                if bowl is None:
                    TalkingMotion(f"I can not find the Metalbowl.").perform()
                    TalkingMotion(f"I will skip pouring and place the objects on the table").perform()
                    if object_type in CUTLERY:
                        # TODO: nach Variante anpassen
                        x_pos += 0.3
                        # place_pose.pose.position.x += 0.3
        if bowl is not None:
            if object_type in ["Cerealbox", "Cronybox", "Milkpack", "MilkpackLactoseFree", "MuesliboxVitalis"]:
                # TODO: Werte anpassen
                NavigateAction([bowl.pose.position.x, NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                               NavigatePose.KITCHEN_TABLE.value.pose.orientation).resolve().perform()
                angle = 115
                try:
                    if robot.get_pose().pose.position.x > bowl.pose.position.x:
                        direction = "right"
                    else:
                        direction = "left"
                    PouringAction([bowl.pose], [Arms.LEFT], [direction], [angle]).resolve().perform()
                except EnvironmentUnreachable:
                    NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                                           robot.get_pose().pose.position.y + 0.3, 0],
                                                          NavigatePose.SHELF.value.pose.orientation)]).resolve().perform()
                    ParkArmsAction([Arms.LEFT]).resolve().perform()
                    MoveTorsoAction([0.12]).resolve().perform()
                    NavigateAction([Pose([NavigatePose.KITCHEN_TABLE.value.pose.position.x + 0.2,
                                          NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                                         NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
                    object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.21],
                                                   NavigatePose.KITCHEN_TABLE.value.pose.orientation))
                    bowl = get_bowl(object_desig)
                    if bowl is not None:
                        NavigateAction([bowl.pose.position.x, NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                                       NavigatePose.KITCHEN_TABLE.value.pose.orientation).resolve().perform()
                        try:
                            if robot.get_pose().pose.position.x > bowl.pose.position.x:
                                direction = "right"
                            else:
                                direction = "left"
                            PouringAction([bowl.pose], [Arms.LEFT], [direction], [angle]).resolve().perform()
                        except EnvironmentUnreachable:
                            NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                                                   robot.get_pose().pose.position.y + 0.3, 0],
                                                                  NavigatePose.SHELF.value.pose.orientation)]).resolve().perform()
                            ParkArmsAction([Arms.LEFT]).resolve().perform()
                            TalkingMotion(f"Pouring will be skipped").perform()
                            TalkingMotion(f"i can not reach the bowl on the table").perform()
                if bowl is not None:
                    # Move away from the table
                    NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                                           robot.get_pose().pose.position.y + 0.3, 0],
                                                          NavigatePose.SHELF.value.pose.orientation)]).resolve().perform()
                    ParkArmsAction([Arms.LEFT]).resolve().perform()
                else:
                    TalkingMotion(f"Pouring will be skipped").perform()
                    TalkingMotion(f"i can not find the bowl on the table").perform()
            else:
                x_pos = bowl.pose.position.x + 0.2
    NavigateAction([Pose([x_pos, NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                         NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
    TalkingMotion("Placing").perform()
    if first_placing:
        PlaceAction(objects_list[index], [Pose([x_pos, NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0.75])],
                    [grasp], [Arms.LEFT], [True]).resolve().perform()
    else:
        PlaceGivenObjectAction([object_type], [Arms.LEFT],
                               [Pose([x_pos, NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0.75])],
                               [grasp]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    if object_type == "Metalbowl":
        x_pos += 0.6
    else:
        x_pos += 0.25
    # navigates back if a next object exists
    if index + 1 < len(objects_list):
        NavigateAction(
            Pose([NavigatePose.SHELF.value.pose.position.x - 0.8, NavigatePose.SHELF.value.pose.position.y, 0],
                 NavigatePose.SHELF.value.pose.orientation)).resolve().perform()
        NavigateAction(NavigatePose.SHELF.value).resolve().perform()


def remove_objects(value):
    """
    removes already transported objects from the list
    :param value: the object that should be removed from the list
    """
    # remove all objects that were seen and transported so far
    if value.obj_type in wished_sorted_obj_list:
        wished_sorted_obj_list.remove(value.obj_type)
    # if the type is cutlery, the real type is not easily reproducible.
    # remove one cutlery object of the list with the highest chance that it was found and transported.
    if value.obj_type == "Cerealbox":
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


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


with (real_robot):
    try:
        plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        ParkArmsAction([Arms.LEFT]).resolve().perform()

        # navigate from door to shelf
        NavigateAction([NavigatePose.DOOR.value]).resolve().perform()
        NavigateAction([NavigatePose.CORRIDOR.value]).resolve().perform()
        NavigateAction([Pose([NavigatePose.CORRIDOR.value.pose.position.x,
                              NavigatePose.SHELF.value.pose.position.y, 0],
                             NavigatePose.CORRIDOR.value.pose.orientation)])
        NavigateAction([NavigatePose.SHELF.value]).resolve().perform()

        # perceive objects
        MoveTorsoAction([0.2]).resolve().perform()
        obj_desig = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.21],
                                    NavigatePose.SHELF.value.pose.orientation))
        sorted_obj = sort_objects(obj_desig, wished_sorted_obj_list)
        print(sorted_obj[0].obj_type)
        if sorted_obj[0].obj_type != "Metalbowl":
            # navigate to shelf
            NavigateAction([NavigatePose.SHELF.value]).resolve().perform()
            bowl_obj_desig = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.21],
                                             NavigatePose.SHELF.value.pose.orientation))
            sorted_obj = sort_objects(bowl_obj_desig, wished_sorted_obj_list)
            if sorted_obj[0].obj_type != "Metalbowl":
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
                TalkingMotion(f"Can you please give me the Metalbowl in the shelf?").perform()
                rospy.sleep(4)
                MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
                place_objects(False, ["Metalbowl"], 0, Grasp.TOP)
                # navigate_to(4.3, 4.9, "shelf")
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
            NavigateAction(NavigatePose.SHELF.value).resolve().perform()
            MoveTorsoAction([0.2]).resolve().perform()
            new_object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.21],
                                               NavigatePose.SHELF.value.pose.orientation))
            new_sorted_obj = sort_objects(new_object_desig, wished_sorted_obj_list)
            pickup_and_place_objects(new_sorted_obj)
            # failure handling part 2
            final_sorted_obj = sorted_obj + new_sorted_obj
            if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
                try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.21],
                                NavigatePose.SHELF.value.pose.orientation))
                print("second Check")
                for value in final_sorted_obj:
                    # remove all objects that were seen and transported so far
                    remove_objects(value)
                for val in range(len(wished_sorted_obj_list)):
                    grasp = Grasp.FRONT
                    if wished_sorted_obj_list[val] in (["Metalbowl"] + CUTLERY):
                        grasp = Grasp.TOP
                    print(f"next object is: {wished_sorted_obj_list[val]}")
                    TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} "
                                  f"in the shelf?").perform()
                    rospy.sleep(4)
                    MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
                    place_objects(False, wished_sorted_obj_list, val, grasp)
        rospy.loginfo("Done!")
        TalkingMotion("Done").perform()
