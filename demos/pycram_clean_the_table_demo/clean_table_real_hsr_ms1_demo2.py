from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy


world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([0, 0, 0]),
                color=[0, 1, 0, 1])

giskardpy.init_giskard_interface()

cereal_desig = BelieveObject(names=["cereal"])
robot_desig = BelieveObject(names=["hsr"])

RobotStateUpdater("/tf", "/giskard_joint_states")


def move_and_detect():
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([3.9, 2.2, 0])]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()
    LookAtAction(targets=[Pose([4.9, 2.2, 0.18])]).resolve().perform()
    TalkingMotion("Perceiving now").resolve().perform()
    object_desig = DetectAction(BelieveObject(types=[cereal.type]), technique='all').resolve().perform()

    return object_desig

def open_gripper():
        """ Opens the gripper of the HSR """
        pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                  queue_size=10)
        rate = rospy.Rate(10) 
        rospy.sleep(2)
        msg = GripperApplyEffortActionGoal() # sprechen joint gripper_controll_manager an, indem wir goal publishen type den giskard fürs greifen erwartet
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


with real_robot:
    TalkingMotion("Starting Demo").resolve().perform()
    object_desig = move_and_detect()

    obj_list = [object_desig['Cerealbox'], object_desig['Pringleschipscan'], object_desig['Milkpack'],
                object_desig['Crackerbox'], object_desig['Metalmug']]

    giskardpy.initial_adding_objects()
    giskardpy.spawn_kitchen()
    
    # turning head up, to avoid collision of head with gripper
    LookAtAction(targets=[Pose([4.9, 2.2, 1.1])]).resolve().perform()

    for obj in obj_list:
        new_pose = obj.pose
        TalkingMotion("Navigating now").resolve().perform()
        NavigateAction(target_locations=[Pose([4.2, new_pose.position.y, 0])]).resolve().perform()

        TalkingMotion("Moving my arm now").resolve().perform()
        PickUpAction(obj, ["left"], ["front"]).resolve().perform()

        TalkingMotion("Picked up, dropping in 3").resolve().perform()
        TalkingMotion("2").resolve().perform()
        TalkingMotion("1").resolve().perform()
        open_gripper()
        rospy.sleep(5)
        ParkArmsAction([Arms.LEFT]).resolve().perform()

    TalkingMotion("I am done").resolve().perform()

