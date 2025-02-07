from typing_extensions import Optional

from pycram.external_interfaces import giskard
from pycram.failures import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.language import Code
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

fts = ForceTorqueSensor(robot_name='hsrb')

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the popcorn table
table_pose = 4.7

# name of the dishwasher handle and dishwasher door
handle_name = "iai_kitchen/sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"

# Intermediate positions for a safer navigation
move_to_the_middle_table_pose = [2.2, 1.98, 0]
move_to_the_middle_dishwasher_pose = [2.2, -0.1, 0]

# Initialize the Bullet world for simulation
world = BulletWorld()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

# robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")
apart_desig = BelieveObject(names=["kitchen"])


class NavigatePose(Enum):
    DOOR = Pose([-1.58, 0, 0], [0, 0, 0, 1])
    CORRIDOR = Pose([3.32, 1.04, 0], [0, 0, 0.7, 0.7])
    KITCHEN_TABLE = Pose([4.49, 5.34, 0], [0, 0, -0.7, 0.7])  # +- 0.2 for left/ right on x
    BEFORE_KITCHEN = Pose([1.67, 6.15, 0], [0, 0, 0.7, 0.7])
    IN_KITCHEN = Pose([1.27, 8.66, 0], [0, 0, 0, 1])
    DISHWASHER_CLOSED = Pose([4.55, 8.75, 0], [0, 0, -0.7, 0.7])
    DISHWASHER_RIGHT = Pose([3.83, 8.4, 0], [0, 0, 0, 1])
    # DISHWASHER_LEFT = Pose([4.55, 8.75, 0], [0, 0, 1, 0])
    SHELF = Pose([4.62, 5.95, 0], [0, 0, 0, 1])


class PlacingXPose(Enum):
    """
    Differentiate the x pose for placing
    """
    CUTLERY = 4.4  # 2.376
    SPOON = 4.4  # 2.376
    FORK = 4.4  # 2.376
    PLASTICKNIFE = 4.4  # 2.376
    KNIFE = 4.4  # 2.376
    METALBOWL = 4.55  # 2.83
    METALMUG = 4.47  # 2.79
    METALPLATE = 4.55  # 2.8


class PlacingYPose(Enum):
    """
    Differentiate the y pose for placing
    """
    CUTLERY = 8.3  # -1.59 # mitte
    SPOON = 8.3  # -1.59
    FORK = 8.3  # -1.59
    PLASTICKNIFE = 8.3  # -1.59
    KNIFE = 8.3  # -1.59
    METALBOWL = 8.15  # -1.73
    METALMUG = 8.2  # -1.75
    METALPLATE = 8.25  # -1.65


def pickup_object(object: Object):
    global table_pose, CUTLERY
    grasp = Grasp.FRONT

    object_pose = object.pose.position
    print(object_pose)

    if object.obj_type in CUTLERY or object.obj_type == "Metalbowl":
        grasp = Grasp.TOP
        MoveTorsoAction([0.4]).resolve().perform()
    else:
        MoveTorsoAction([0.2]).resolve().perform()

    if object.obj_type == "Metalplate":
        TalkingMotion("Can you please give me the plate on the table.").perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        rospy.sleep(3)

        TalkingMotion("Grasping.").perform()

        MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
    else:
        if object.obj_type in CUTLERY:  # and object.pose.position.y > table_pose + 0.125:
            object.pose.position.z = 0.75
        # change object x pose if the grasping pose is too far in the table
        # object.pose.position.y -= 0.1
        if object.obj_type == "Metalbowl":
            object.pose.position.z = 0.76
        TalkingMotion("Picking up from: " + (str(grasp)[6:]).lower()).perform()
        try_pick_up_c(robot, object, grasp)

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                           robot.get_pose().pose.position.y + 0.3, 0],
                                          NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
    MoveTorsoAction([0]).resolve().perform()

    if object.obj_type in CUTLERY:
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 4.35, 0.35],
                                       NavigatePose.KITCHEN_TABLE.value.pose.orientation))
        if object_found(object_desig, str(object.obj_type)):
            new_object = get_object(object_desig, str(object.obj_type))
            MoveTorsoAction([0.4]).resolve().perform()
            try_pick_up_c(robot, new_object, grasp)
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                                   robot.get_pose().pose.position.y + 0.3, 0],
                                                  [0, 0, 0.7, 0.7])]).resolve().perform()
            MoveTorsoAction([0]).resolve().perform()


def place_object(object: Object):
    x_y_z_pos = get_pos(str(object.obj_type).upper())

    x_pos = x_y_z_pos[0]
    y_pos = x_y_z_pos[1]

    TalkingMotion("Placing").perform()
    grasp = Grasp.FRONT

    MoveTorsoAction([0.3]).resolve().perform()

    PlaceAction(object, [Pose([x_pos, y_pos, 0.7])], [grasp], [Arms.LEFT],
                [False]).resolve().perform()

    # For the safety in cases where the HSR is not placing, better drop the object to not colide with the kitchen
    # drawer when moving to parkArms arm config
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()


def pickup_and_place(objects_list: list):
    NavigateAction([Pose([objects_list[0].pose.position.x, robot.get_pose().pose.position.y, 0],
                         NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
    for value in range(len(objects_list)):
        pickup_object(objects_list[value])
        # turn around
        NavigateAction([Pose(robot.get_pose().pose.position,
                             NavigatePose.BEFORE_KITCHEN.value.pose.orientation)]).resolve().perform()
        # navigate to dishwasher
        NavigateAction([NavigatePose.BEFORE_KITCHEN.value]).resolve().perform()
        NavigateAction([NavigatePose.IN_KITCHEN.value]).resolve().perform()
        NavigateAction([NavigatePose.DISHWASHER_RIGHT.value]).resolve().perform()
        place_object(objects_list[value])
        if value + 1 < len(objects_list):
            NavigateAction([Pose(NavigatePose.IN_KITCHEN.value.pose.position,
                                 NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
            NavigateAction([Pose(NavigatePose.BEFORE_KITCHEN.value.pose.position,
                                 NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
            NavigateAction([Pose([objects_list[value + 1].pose.position.x,
                                  NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                                 NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()


# TODO: implement function for droping objects in trash can
# def throw_object():


def get_pos(obj_type: str):
    """
      Getter for x and y value for placing the given object type.

      :param obj_type: type of object, which x and y pose for placing we want
      :return: the tupel of x and y value for placing that object
      """
    x_val = PlacingXPose[obj_type].value
    y_val = PlacingYPose[obj_type].value
    return x_val, y_val


def navigate_and_detect(location_name: NavigatePose):
    """
    Navigates to a certain location and perceives.

    :param location_name: the location the robot navigates to
    :return: tupel of State and dictionary of found objects in the FOV
    """
    TalkingMotion("Navigating").perform()
    # # Navigate out of kitchen area
    # NavigateAction([Pose(NavigatePose.IN_KITCHEN.value.pose.position,
    #                      NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
    # NavigateAction([Pose(NavigatePose.BEFORE_KITCHEN.value.pose.position,
    #                      NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()

    if location_name == NavigatePose.SHELF:
        NavigateAction([NavigatePose.SHELF.value]).resolve().perform()
        MoveTorsoAction([0.2]).resolve().perform()
        object_desig = try_detect(Pose([robot.get_pose().pose.position.x, 5.925, 0.21],
                                       NavigatePose.SHELF.value.pose.orientation))
        objects_list = get_objects(object_desig)
    elif location_name == NavigatePose.KITCHEN_TABLE:
        # perceive left side
        NavigateAction([Pose([NavigatePose.KITCHEN_TABLE.value.pose.position.x + 0.2,
                              NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                             NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig1 = try_detect(Pose([robot.get_pose().pose.position.x, 4.35, 0.35],
                                        NavigatePose.KITCHEN_TABLE.value.pose.orientation))
        objects_list1 = get_objects(object_desig1)

        # perceive right side
        NavigateAction([Pose([NavigatePose.KITCHEN_TABLE.value.pose.position.x - 0.2,
                              NavigatePose.KITCHEN_TABLE.value.pose.position.y, 0],
                             NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
        MoveTorsoAction([0.12]).resolve().perform()
        object_desig2 = try_detect(Pose([robot.get_pose().pose.position.x, 4.35, 0.35],
                                        NavigatePose.KITCHEN_TABLE.value.pose.orientation))
        objects_list2 = get_objects(object_desig2)
        objects_list = []
        for object in objects_list1 + objects_list2:
            if object not in objects_list:
                objects_list.append(object)
    else:
        raise ValueError(f'Incorrect location name: {location_name}.')

    return objects_list


def failure_handling1(sorted_obj: list):
    """
    Part 1 of the failure handling consists of perceiving a second time and pick up and placing the seen objects.

    :param sorted_obj: list of seen objects.
    :return: list of seen objects in the second round. Empty list when nothing perceived or all objects already found.
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list, move_to_the_middle_table_pose
    new_objects_list = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive, pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            if value.obj_type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.obj_type)
        # todo should not always navigate to middle pose. think about a case where she stands already infront
        #  of the table and didn't perceived anything.
        new_objects_list = navigate_and_detect(NavigatePose.KITCHEN_TABLE)
        pickup_and_place(new_objects_list)
    return new_objects_list


def failure_handling2(sorted_obj: list, new_sorted_obj: list):
    """
    Part 2 of the failure handling, when object is not seen again, the robot is asking for human support.

    :param sorted_obj: list of already seen and transported objects
    :param new_sorted_obj: list of objects that were seen in the first part of the failure handling
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list
    # failure handling part 2
    final_sorted_obj = sorted_obj + new_sorted_obj
    if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        NavigateAction([Pose(NavigatePose.IN_KITCHEN.value.pose.position,
                             NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
        NavigateAction([Pose(NavigatePose.BEFORE_KITCHEN.value.pose.position,
                             NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
        NavigateAction([NavigatePose.KITCHEN_TABLE.value]).resolve().perform()

        print("second Check")
        for value in final_sorted_obj:
            # remove all objects that were seen and transported so far
            if value.obj_type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.obj_type)

        for val in range(len(wished_sorted_obj_list)):
            print(f"next object is: {wished_sorted_obj_list[val]}")
            TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} on the table?").perform()
            rospy.sleep(4)
            TalkingMotion("Grabing.").perform()
            MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()

            ParkArmsAction([Arms.LEFT]).resolve().perform()

            # turn around
            NavigateAction([Pose(robot.get_pose().pose.position,
                                 NavigatePose.BEFORE_KITCHEN.value.pose.orientation)]).resolve().perform()
            NavigateAction([NavigatePose.BEFORE_KITCHEN.value]).resolve().perform()
            NavigateAction([NavigatePose.IN_KITCHEN.value]).resolve().perform()
            NavigateAction([NavigatePose.DISHWASHER_RIGHT.value]).resolve().perform()

            x_y_z_pos = get_pos(wished_sorted_obj_list[val].upper())

            x_pos = x_y_z_pos[0]
            y_pos = x_y_z_pos[1]

            TalkingMotion("Placing").perform()
            grasp = Grasp.FRONT

            # todo add placing of plate in PlaceGivenObjAction
            if wished_sorted_obj_list[val] == "Metalplate":
                # PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"],
                # [Pose([x_pos, y_pos, 0.3])], [grasp], False).resolve().perform()
                TalkingMotion("Please take the plate and place it in the dishwasher").perform()
                rospy.sleep(2)
                TalkingMotion("Droping object now").perform()
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            else:
                PlaceGivenObjectAction([wished_sorted_obj_list[val]], [Arms.LEFT],
                                       [Pose([x_pos, y_pos, 0.7])], [grasp]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

            # navigates back if a next object exists
            if val + 1 < len(wished_sorted_obj_list):
                NavigateAction([Pose(NavigatePose.IN_KITCHEN.value.pose.position,
                                     NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
                NavigateAction([Pose(NavigatePose.BEFORE_KITCHEN.value.pose.position,
                                     NavigatePose.KITCHEN_TABLE.value.pose.orientation)]).resolve().perform()
                NavigateAction([NavigatePose.KITCHEN_TABLE.value]).resolve().perform()


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


# Main interaction sequence with real robot
with (real_robot):
    try:
        plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

        # navigate from door to dishwasher
        NavigateAction([NavigatePose.DOOR.value]).resolve().perform()
        NavigateAction([NavigatePose.CORRIDOR.value]).resolve().perform()
        NavigateAction([NavigatePose.BEFORE_KITCHEN.value]).resolve().perform()
        NavigateAction([NavigatePose.IN_KITCHEN.value]).resolve().perform()
        NavigateAction([NavigatePose.DISHWASHER_CLOSED.value]).resolve().perform()

        # MoveJointsMotion(["wrist_roll_joint"], [-1.5]).perform()
        # MoveJointsMotion(["arm_roll_joint"], [0]).perform()
        # giskard.dishwasher_test(handle_name, 'sink_area_dish_washer_door_joint', door_name)

        # OpenDishwasherAction(handle_name, door_name, 0.6, 1.4, [Arms.LEFT]).resolve().perform()

        TalkingMotion("Can you please open the dishwasher and pull out the lower rack").perform()

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

        # detect objects
        object_desig_list = navigate_and_detect(NavigatePose.KITCHEN_TABLE)

        # sort objects based on distance and which we like to keep
        sorted_obj = sort_objects_euclidian(robot, object_desig_list, wished_sorted_obj_list)
        # sorted_obj = sort_objects(object_desig_list, wished_sorted_obj_list)

        # picking up and placing objects
        pickup_and_place(sorted_obj)

        new_obj_desig = failure_handling1(sorted_obj)
        failure_handling2(sorted_obj, new_obj_desig)

        rospy.loginfo("Done!")
        TalkingMotion("Done").perform()
