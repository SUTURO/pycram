import rospy
from move_base_msgs.msg import MoveBaseAction
from roslibpy import actionlib, tf
from sensor_msgs.msg import JointState

from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.enums import ObjectType
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from pycram.enums import ImageEnum as ImageEnum
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter

# test
world = BulletWorld()
# v = VizMarkerPublisher()
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
print("before giskard")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

tf_topic = "/tf"
joint_state_topic = "/iai_kitchen/joint_states"

kitchen.set_joint_state("living_room:arena:door_origin_revolute_joint",0)
kitchen.set_joint_state("living_room:arena:door_handle_inside_joint", 0 )
joint_state = kitchen.get_complete_joint_state()
print(joint_state)
pub = rospy.Publisher(joint_state_topic, JointState, queue_size=10, latch=True)
msg = JointState()
pub.publish(joint_state)
print(kitchen.joints)



