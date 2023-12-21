import rospy
from geometry_msgs.msg import PoseStamped
from robokudo_msgs.msg import QueryActionGoal

from pycram.designators.action_designator import DetectAction, LookAtAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.external_interfaces import robokudo
from pycram.helper import axis_angle_to_quaternion
from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot, semi_real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
import talk_actions

world = BulletWorld("")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

kitchen_ori = axis_angle_to_quaternion([0, 0, 1], 90)


world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
#kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
#kitchen_desig = ObjectDesignatorDescription(names=["kitchen"], pose=Pose([0, 0, 0], kitchen_ori))
milk = Object("Milkpack", "milk", "milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])

giskardpy.init_giskard_interface()
# giskardpy.sync_worlds()
# RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")


#with real_robot:
# Perception

# LookAtAction(targets=[human_pose]).resolve().perform()
# giskardpy.move_head_to_human()
