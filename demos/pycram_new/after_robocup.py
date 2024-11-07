import math

import rospy
from geometry_msgs.msg import PoseStamped
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.datastructures.dataclasses import Color
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import TalkingMotion
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.designators.object_designator import *
from std_msgs.msg import String, Bool
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter

# new imports
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.datastructures.enums import ImageEnum as ImageEnum, Arms
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.worlds.bullet_world import BulletWorld

extension = ObjectDescription.get_file_extension()

# def monitor_func():
#     der = fts.get_last_value()
#     if abs(der.wrench.force.x) > 10.30:
#         return SensorMonitoringCondition
#     return False

world = BulletWorld(WorldMode.DIRECT)
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot_color = Color(R=0.6, G=0.6, B=0.6, A=1)
robot.set_color(robot_color)

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_5.urdf")
# apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment{extension}")
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# important Publishers
move = PoseNavigator()
talk = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()


# Declare variables for humans
guest1 = HumanDescription("Lisa", fav_drink="water")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)


def pakerino(torso_z=0.05, config=None):
    """
    replace function for park arms, robot takes pose of configuration of joint
    """

    if not config:
        config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2,
                  'wrist_flex_joint': -1.5, 'wrist_roll_joint': 0, 'head_pan_joint': 0}

    #giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("[32mParking done")


def convert_to_radians(angle):
    radians = (angle * math.pi) / 180
    return radians

def turn_quaternion(quaternion, angle):
    angle = convert_to_radians(angle)
    quaternion = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]

    return quaternion


def demo(step):
    with real_robot:
            global wait_bool
            global callback
            global doorbell
            global guest1
            global guest2

            move = PoseNavigator()

            print("detecting")

            base_orientation = [0.0, 0.0, 0.0, 1.0]
            modified_orientation = turn_quaternion(quaternion=base_orientation, angle=90)
            pose = Pose(position=[2.585, 5.77, 0.86], orientation=modified_orientation)
            # object_desig[0].set_pose(pose)

            other_pose = Pose(position=[2.585, 5.77, 0.86], orientation=modified_orientation)

            box_object = Object("milk", ObjectType.MILK, "milk.stl", pose=pose,
                                color=Color(1, 0, 0, 1))

            box_desig = ObjectDesignatorDescription.Object(box_object.name, ObjectType.MILK, box_object)

            ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

            # PickUpAction(object_designator_description=[box_desig], arms=[Arms.LEFT], grasps=[Grasp.FRONT]).resolve().perform()

            PlaceAction(box_desig, [other_pose], Grasp.FRONT, [Arms.LEFT]).resolve().perform()


            print("end")
            #rospy.sleep(10)



demo(0)
