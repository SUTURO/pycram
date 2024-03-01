from pycram.process_module import real_robot, semi_real_robot, with_real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater

# worked on 8.12.2023 together with Mohammad
# worked on 10.12.2023
# worked on 11.12.2023

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])

# turn robot and some object for 90 degrees
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)

# Initialize objects in BulletWorld
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()

kitchen = Object("kitchen", "environment", "kitchen.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

milk = Object("Milkpack", "milk", "milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])
cereal = Object("Cerealbox", "cereal", "breakfast_cereal.stl", pose=Pose([-2.5, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
metalmug = Object("Metalmug", "metalmug", "bowl.stl", pose=Pose([-2.9, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
cheezels = Object("Crackerbox", "cheezels", "breakfast_cereal.stl", pose=Pose([-2.3, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
pringles = Object("Pringleschipscan", "pringles", "milk.stl", pose=Pose([-3.1, 2.3, 0.43]), color=[0, 1, 0, 1])


milk_desig = BelieveObject(names=["Milkpack"])
cereal_desig = BelieveObject(names=["Cerealbox"])
metalmug_desig = BelieveObject(names=["Metalmug"])
cheezels_desig = BelieveObject(names=["Crackerbox"])
pringles_desig = BelieveObject(names=["Pringleschipscan"])

# initialize and sync giskard
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")


with semi_real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    # navigate to table
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], object_orientation)]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], object_orientation)]).resolve().perform()

    # take perceiving pose
    LookAtAction(targets=[Pose([-2.6, 2.0, 0.5])]).resolve().perform()
    object_desig = DetectAction(milk_desig, technique='all').resolve().perform()
    object_list= [object_desig["Cerealbox"], object_desig["Crackerbox"], object_desig["Pringleschipscan"], object_desig["Metalmug"], object_desig["Milkpack"]]

    # pick up and drop all five objects
    for index in range(len(object_list)):
        # use robokudo to perceive object pose
       # object_desig = DetectAction(object_list[index]).resolve().perform()
        new_pose = object_list[index].pose
        print(new_pose)
        # navigate hsr to the next object to pick up
        NavigateAction(target_locations=[Pose([new_pose.position.x, 1.5, 0], object_orientation)]).resolve().perform()
        # differentiate the grasping movement between metalmug and other objects
        if object_list[index].name == "Metalmug":
            PickUpAction(object_designator_description=object_list[index],
                         arms=["left"],
                         grasps=["top"]).resolve().perform()
            print("grasp from top")
        elif object_list[index].name == "Cerealbox" or object_list[index].name == "Crackerbox":
            PickUpAction(object_designator_description=object_list[index],
                         arms=["left"],
                         grasps=["left"]).resolve().perform()
            print("grasp from left")
        else:
            PickUpAction(object_designator_description=object_list[index],
                         arms=["left"],
                         grasps=["front"]).resolve().perform()
            print("grasp from front")

        # inform user before dropping the object
        TalkingMotion("I drop the object now!").resolve().perform()
        print("I drop the object now!")

        # drop/place the object
        place_pose = Pose([new_pose.position.x - 0.3, new_pose.position.y, new_pose.position.z])
        PlaceAction(object_list[index], [place_pose], ["left"]).resolve().perform()
        print("placing finished")
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        # go back for perceiving the next object
        NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], object_orientation)]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], object_orientation)]).resolve().perform()
    print("finished")
# MoveGripperMotion("open", "left", allow_gripper_collision=True).resolve().perform()



#manipulation perceive pose
#perceived_items[objectdesig] = perception call
#     spawn perceived_items
#     syn.world giskard
#     add.mesh to giskard
#     loop for items
#        wenn items[i] == item1 dann
#         item1 = items[i]
#        ...
#      item1 pick up
#      drop item / place item1 in simulation
#      ...
#
