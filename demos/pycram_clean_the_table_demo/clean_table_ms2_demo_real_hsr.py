import time
from pycram.designators.location_designator import AccessingLocation
from pycram.process_module import real_robot, semi_real_robot, with_real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater

# worked on 27.12
# worked on 28.12

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])

# turn robot and some object for 90 degrees
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
transport_orientation = axis_angle_to_quaternion([0, 0, 1], 270)

# Initialize objects in BulletWorld
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot_pose = robot.get_pose()

kitchen = Object("kitchen", "environment", "kitchen.urdf")
kitchen_desig = BelieveObject(names=["kitchen"])

fork = Object("Fork", "fork", "spoon.stl", pose=Pose([-2.8, 2.3, 0.368], object_orientation), color=[1, 0, 0, 1])
spoon = Object("Spoon", "spoon", "spoon.stl", pose=Pose([-2.5, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
metalmug = Object("Metalmug", "metalmug", "bowl.stl", pose=Pose([-3.1, 2.3, 0.39]), color=[0, 1, 0, 1])
plate = Object("Plate", "plate", "board.stl", pose=Pose([-2.2, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
bowl = Object("Bowl", "bowl", "bowl.stl", pose=Pose([-3.4, 2.3, 0.39]), color=[0, 1, 0, 1])

fork_desig = BelieveObject(names=["Fork"])
spoon_desig = BelieveObject(names=["Spoon"])
metalmug_desig = BelieveObject(names=["Metalmug"])
plate_desig = BelieveObject(names=["Plate"])
bowl_desig = BelieveObject(names=["Bowl"])

# initialize and sync giskard
giskardpy.init_giskard_interface()
#giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")


def move_and_detect():
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    #TODO knowledge einbinden für Tischposition.
    # Vielleicht auch mit Costmap und knowledge gibt immer position von
    # Tischmitte? Offset berechnen
    NavigateAction(target_locations=[Pose([-2.6, 0.1, 0], object_orientation)]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], object_orientation)]).resolve().perform()

    # NavigateAction(target_locations=[Pose([3.9, 2.2, 0])]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()
    LookAtAction(targets=[Pose([-2.6, 2.0, 0.3])]).resolve().perform()
    # LookAtAction(targets=[Pose([4.9, 2.2, 0.18])]).resolve().perform()
    TalkingMotion("Perceiving now").resolve().perform()
    object_desig = DetectAction(BelieveObject(types=[bowl.type]), technique='all').resolve().perform()
    time.sleep(1)
    LookAtAction(targets=[Pose([-2.6, 2.0, 0.7])]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    return object_desig


def open_gripper():
    """ Opens the gripper of the HSR """
    pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                  queue_size=10)
    rate = rospy.Rate(10)
    rospy.sleep(2)
    msg = GripperApplyEffortActionGoal()  # sprechen joint gripper_controll_manager an, indem wir goal publishen type den giskard fürs greifen erwartet
    msg.goal.effort = 0.8
    pub_gripper.publish(msg)


def close_gripper():
    """ Closes the gripper of the HSR """
    pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                  queue_size=10)
    rate = rospy.Rate(10)
    rospy.sleep(2)
    msg = GripperApplyEffortActionGoal()
    msg.goal.effort = -0.8
    pub_gripper.publish(msg)


def sort_obj_list(obj_dict):
    wished_sorted_obj_list = ["Bowl", "Metalmug", "Fork", "Spoon", "Plate"]
    obj_list = []
    sorted_obj_list = []
    for key, value in obj_dict.items():
        if key in wished_sorted_obj_list:
            obj_list.append(value)

    if len(obj_list) != len(sorted_obj_list):
        TalkingMotion("Could not perceive all needed objects")

    j = 0
    helper = []
    for object in obj_list:
        helper.append(object.name)
    print("helper: ", helper)
    for i in range(0, len(wished_sorted_obj_list)):
        print("index: ", i)
        if wished_sorted_obj_list[i] in helper:
            sorted_obj_list.insert(j, obj_list[helper.index(wished_sorted_obj_list[i])])
            j = j + 1

    for obj in sorted_obj_list:
        print("Objects in list: ", obj.name)
    return sorted_obj_list

def try_pick_up(obj, grasps):
    try:
        PickUpAction(obj, ["left"], [grasps]).resolve().perform()
    except:
        print("try pick up again")
        TalkingMotion("Try pick up again")
        NavigateAction([Pose([robot_pose.position.x - 0.3, robot_pose.position.y, robot_pose.position.z],
                             robot_pose.orientation)]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        object_desig = DetectAction(BelieveObject(types=[ObjectType.MILK]), technique='all').resolve().perform()
        # TODO nur wenn key (name des vorherigen objektes) in object_desig enthalten ist
        new_object = object_desig[obj.name]
        try:
            PickUpAction(new_object, ["left"], [grasps]).resolve().perform()

        except:
            TalkingMotion(f"Can you pleas give me the {obj.name} object on the table? Thanks")
            TalkingMotion(f"Please push down my hand, when I can grab the {obj.name}.")


with semi_real_robot:
    TalkingMotion("Starting Demo").resolve().perform()

    # TODO: Öffnen von Geschirrspüler handle_desig wirft Fehler
    # handle_desig = ObjectPart(names=["kitchen_2/sink_area_dish_washer_door"], part_of=kitchen_desig.resolve()).resolve()
    # print(f"handle_desig: {handle_desig}")
    # dishwasher_open_loc = AccessingLocation(handle_desig=handle_desig, robot_desig=robot_desig).resolve()
    #
    # NavigateAction([dishwasher_open_loc.pose]).resolve().perform()
    #
    # OpenAction(object_designator_description=handle_desig, arms=[dishwasher_open_loc.arms[0]]).resolve().perform()
    # print("open dishwasher")

    object_desig = move_and_detect()
    sorted_obj = sort_obj_list(object_desig)

    giskardpy.initial_adding_objects()
    giskardpy.spawn_kitchen()

    for obj in sorted_obj:
        new_pose = obj.pose
        TalkingMotion("Navigating now").resolve().perform()
        #TODO: knowledge Tischposition
        #NavigateAction(target_locations=[Pose([new_pose.position.x, 1.5, 0], object_orientation)]).resolve().perform()
        NavigateAction(target_locations=[Pose([4.2, new_pose.position.y, 0])]).resolve().perform()

        TalkingMotion("Moving my arm now").resolve().perform()
        print("Object name: ", obj.name)
        # TODO: Aufruf von knowledge für Tischposition popcorn
        target_location = Pose([3.5, new_pose.position.y, new_pose.position.z], transport_orientation)

        if obj.name == "Plate":
            TalkingMotion("Can you pleas give me the last object on the table, the plate? Thanks")
            TalkingMotion("Please push down my hand, when I can grab the plate.")
            print("pick up plate")
            #TODO: erfasse gripper state für push down
            MoveGripperMotion("close", "left", allow_gripper_collision=True)

        else:
            try_pick_up(obj, "top")

        rospy.sleep(5)
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        # TODO: knowledge Tischposition popcorn table
        # TODO: Muss hsr in die richtige Richtung schauen???
        NavigateAction([Pose([3.6, new_pose.position.y, 0], transport_orientation)]).resolve().perform()
        PlaceAction(obj, [target_location], ["left"]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        print("placed")

    TalkingMotion("I am done").resolve().perform()
    print("finished")

