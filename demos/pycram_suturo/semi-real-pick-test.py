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
from pycram.external_interfaces.robokudo import queryEmpty, queryHuman
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))

apartment = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([-2.5, 1.5, 0.43]),
                color=[0, 1, 0, 1])

#
cereal_desig = BelieveObject(names=["cereal"])

robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = BelieveObject(names=["hsr"])
apartment_desig = BelieveObject(names=["kitchen"])


#todo make a function that removes giskard stuff without bullet
#giskardpy.removing_of_objects()

#we need to make this work with the real robot otherwise pick up action will not work
RobotStateUpdater("/tf", "/giskard_joint_states")

def move_and_detect(obj):
    NavigateAction(target_locations=[Pose([-2.6, 0.8, 0], object_orientation)]).resolve().perform()
    LookAtAction(targets=[obj.pose]).resolve().perform()
    LookAtAction(targets=[obj.pose]).resolve().perform()
    object_desig = DetectAction(BelieveObject(types=[obj.type]), technique='default').resolve().perform()

    return object_desig


with semi_real_robot:
    cereal_target_pose = Pose([-2.5, 2.3, 0.43], [0, 0, 1, 1])
    TalkingMotion("test").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    dict_desig = move_and_detect(obj=cereal)
    #giskardpy.initial_adding_objects()
    #giskardpy.spawn_kitchen()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    TalkingMotion("Trying to pick up").resolve().perform()
    PickUpAction((dict_desig["cereal"]), ["left"], ["top"]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    #NavigateAction(target_locations=[cereal_target_pose]).resolve().perform()
    #PlaceAction(dict_desig['cereal'], [Pose(-2.7, 2.3, 0.43)], ["left"]).resolve().perform()


    PlaceAction(dict_desig["cereal"], [cereal_target_pose], ["left"]).resolve().perform()
    giskardpy.remove_object(dict_desig["cereal"].bullet_world_object)
    ParkArmsAction([Arms.LEFT]).resolve().perform()


    # #todo parkarms does not move the torso atm
