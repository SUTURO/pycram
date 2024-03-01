from math import sqrt

import rospy
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot, real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy
import rospy

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot, real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy

from geometry_msgs.msg import Point
# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning



#giskardpy.spawn_kitchen()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()
#giskardpy.removing_of_objects()
#giskardpy.sync_worlds()
robot.set_color([0.5, 0.5, 0.9, 1])
robot_pose = robot.get_pose()

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchen.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

# Define a breakfast cereal object
#breakfast_cereal = Object("breakfast_cereal", "breakfast_cereal", "breakfast_cereal.stl", pose=Pose([4.8, 2.6, 0.87]),color=[0, 1, 0, 1])
# fork = Object("Fork", "fork", "spoon.stl", pose=Pose([-2.8, 2.3, 0.368], object_orientation), color=[1, 0, 0, 1])
# spoon = Object("Spoon", "spoon", "spoon.stl", pose=Pose([-2.5, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
# metalmug = Object("Metalmug", "metalmug", "bowl.stl", pose=Pose([-3.1, 2.3, 0.39]), color=[0, 1, 0, 1])
# plate = Object("Plate", "plate", "board.stl", pose=Pose([-2.2, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
#bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([4.8, 2.6, 0.87]), color=[0, 1, 0, 1])

# Set the world's gravity
#world.set_gravity([0.0, 0.0, 9.81])

def sort_objects(obj_dict, wished_sorted_obj_list):
    sorted_objects= []
    object_tuples = []
    isPlate = True
    isAdded = False
    #print(f"gefundene liste: {obj_dict}")
    first, *remaining = obj_dict
    print(f"type: {type(remaining)} and type of obj_dict: {type(obj_dict)}")
    robot_pose = robot.get_pose()
    for dict in remaining:
        for value in dict.values():
            print(f"value type: {type(value)}")
            distance = sqrt(pow((value.pose.position.x - robot_pose.position.x), 2) +
                         pow((value.pose.position.y - robot_pose.position.y), 2) +
                         pow((value.pose.position.z - robot_pose.position.z), 2))

            print(f"object name: {value.name} and distance: {distance}")

            if isPlate or value.type != "Metalplate":
                object_tuples.append((value, distance))
                if value.type == "Metalplate":
                     isPlate = False

    sorted_object_list = sorted(object_tuples, key= lambda distance: distance[1])

    for (object, distance) in sorted_object_list:
        #todo: reminder types are written with small letter and names now with big beginning letter
        if object.type in wished_sorted_obj_list:
            if object.type == "Metalplate":
                sorted_objects.insert(0, object)
            else:
                sorted_objects.append(object)

    test_list = []
    for test_object in sorted_objects:
         test_list.append(test_object.name)
    print(test_list)

    return sorted_objects

def try_pick_up(obj, grasps):
    try:
        PickUpAction(obj, ["left"], [grasps]).resolve().perform()
    except (EnvironmentUnreachable, GripperClosedCompletely):
        print("try pick up again")
        TalkingMotion("Try pick up again")
        NavigateAction([Pose([robot.get_pose().position.x - 0.3, robot.get_pose().position.y, robot.get_pose().position.z],
                             robot.get_pose().orientation)]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        if EnvironmentUnreachable:
             object_desig = DetectAction(technique='default').resolve().perform()
             # TODO nur wenn key (name des vorherigen objektes) in object_desig enthalten ist
             new_object = object_desig[obj.name]
        else:
             new_object = obj
        try:
            PickUpAction(new_object, ["left"], [grasps]).resolve().perform()

        except:
            TalkingMotion(f"Can you pleas give me the {obj.name} object on the table? Thanks")
            TalkingMotion(f"Please push down my hand, when I can grab the {obj.name}.")

# Main interaction sequence with semi-real robot
with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()
    MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    TalkingMotion("Navigating").resolve().perform()
    NavigateAction(target_locations=[Pose([1.6, 1.8, 0], [0,0,1,0])]).resolve().perform()
    #popcorntable
    LookAtAction(targets=[Pose([0.8, 1.8, 0.21], object_orientation)]).resolve().perform()
    LookAtAction(targets=[Pose([0.8, 1.8, 0.21], object_orientation)]).resolve().perform()
    TalkingMotion("Perceiving").resolve().perform()
    object_desig = DetectAction(technique='default').resolve().perform()
    wished_sorted_obj_list = ["Bowl", "Metalmug", "Fork", "Spoon", "Metalplate", "Cerealbox", "Milkpack"]
    sorted_obj = sort_objects(object_desig, wished_sorted_obj_list)

    y_pos = 1.66
    for value in sorted_obj:
        NavigateAction(target_locations=[Pose([1.6, value.pose.position.y, 0], [0, 0, 1, 0])]).resolve().perform()
        print("first navigation")
        cutlery = ["Spoon","Fork","Knife","Plasticknife"]
        grasp = "front"
        if (value.type in cutlery):
            value.type = "Cutlery"

        if value.type in ["Bowl","Cutlery"]:
            grasp = "top"

        if value.type == "Metalplate":
            print("MetalPlate!!!!!!!!!!!!!!!!!!")
            MoveGripperMotion("open", "left").resolve().perform()
            TalkingMotion("Can you pleas give me the plate on the table.").resolve().perform()
           # TalkingMotion("Please push down my hand, when I can grab the plate.").resolve().perform()
            print("picked up plate")
            time.sleep(3)
            MoveGripperMotion("close", "left").resolve().perform()
        else:
            table_pose = 1.04
            if value.type == "Cutlery" and value.pose.position.x + 0.05 >= table_pose:
                print("adjusted x!!!!")
                value.pose.position.x -= 0.1
            TalkingMotion("Picking Up with: " + grasp).resolve().perform()
            try_pick_up(value, grasp)

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        NavigateAction(target_locations=[Pose([1.8, 1.8, 0], [0, 0, 0, 1])]).resolve().perform() # 1.6 für x
        NavigateAction(target_locations=[Pose([4.1, y_pos, 0], [0, 0, 0, 1])]).resolve().perform()
        TalkingMotion("Placing").resolve().perform()
        #Todo: Objekte in z unterscheiden
        if value.type == "Cutlery":
            z = 0.8
        elif value.type == "Bowl":
            z = 0.84
        elif value.type == "Metalmug":
            z = 0.8
        elif value.type == "Milkpack":
            z = 0.88
        elif value.type == "Cerealbox":
            z = 0.9
        if value.type == "Metalplate":
            PlaceGivenObjAction(["left"], [Pose([4.86, y_pos, 0])]).resolve().perform()
        else:
            PlaceAction(value, ["left"], [grasp], [Pose([4.9, y_pos, z])]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        NavigateAction(target_locations=[Pose([3.9, 2, 0], [0, 0, 1, 0])]).resolve().perform() # 4.1 für x
        if value.type == "Metalplate":
            y_pos += 0.3
        else:
            y_pos += 0.16

    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()



