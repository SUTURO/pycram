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
from pycram.external_interfaces.robokudo import queryEmpty, queryHuman

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))
# human = Object("human", ObjectType.MILK, "human_male.stl", pose=Pose([0, 0, 0]))

# apartment = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
# turn robot and some object for 90 degrees
giskardpy.init_giskard_interface()
# giskardpy.sync_worlds()

object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([0, 0, 0]),
                color=[0, 1, 0, 1])

cereal_desig = BelieveObject(names=["cereal"])

robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = BelieveObject(names=["hsr"])
apartment_desig = BelieveObject(names=["kitchen"])

# todo make a function that removes giskard stuff without bullet
# giskardpy.removing_of_objects()

RobotStateUpdater("/tf", "/giskard_joint_states")


def move_and_detect(obj):
    # NavigateAction(target_locations=[Pose([-2.6, 0.8, 0], object_orientation)]).resolve().perform()
    # LookAtAction(targets=[obj.pose]).resolve().perform()
    # LookAtAction(targets=[obj.pose]).resolve().perform()
    object_desig = DetectAction(BelieveObject(types=[obj.type]), technique='all').resolve().perform()

    return object_desig


with real_robot:
    TalkingMotion("Starting Demo").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([3.9, 2.2, 0])]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()
    LookAtAction(targets=[Pose([4.9, 2.2, 0.18])]).resolve().perform()


    def open_gripper():
        pub_nlp = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                  queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        rospy.sleep(2)
        msg = GripperApplyEffortActionGoal()
        msg.goal.effort = 0.8
        pub_nlp.publish(msg)


    def close_gripper():
        pub_nlp = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                  queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        rospy.sleep(2)
        msg = GripperApplyEffortActionGoal()
        msg.goal.effort = -0.8
        pub_nlp.publish(msg)



    TalkingMotion("Perceiving Now").resolve().perform()
    object_desig = DetectAction(BelieveObject(types=[cereal.type]), technique='all').resolve().perform()

    obj_list = [object_desig['Cerealbox'], object_desig['Pringleschipscan'], object_desig['Milkpack'],
                object_desig['Crackerbox'], object_desig['Metalmug']]

    giskardpy.initial_adding_objects()
    giskardpy.spawn_kitchen()
    LookAtAction(targets=[Pose([4.9, 2.2, 1.1])]).resolve().perform()

    for obj in obj_list:
        new_pose = obj.pose
        TalkingMotion("Navigating now").resolve().perform()
        NavigateAction(target_locations=[Pose([4.2, new_pose.position.y, 0])]).resolve().perform()

        TalkingMotion("Calling Giskard now").resolve().perform()
        PickUpAction(obj, ["left"], ["front"]).resolve().perform()

        TalkingMotion("Picked up Dropping in 3").resolve().perform()
        TalkingMotion("2").resolve().perform()
        TalkingMotion("1").resolve().perform()
        open_gripper()
        rospy.sleep(5)
        ParkArmsAction([Arms.LEFT]).resolve().perform()

    TalkingMotion("I am done").resolve().perform()
