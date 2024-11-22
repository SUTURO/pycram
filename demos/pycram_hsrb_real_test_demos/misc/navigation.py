import numpy as np
import rospy

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal
from pycram.designator import LocationDesignatorDescription
import random


move = PoseNavigator()

table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])


# door_pose
robot_orientation1 = axis_angle_to_quaternion([0, 0, 1], 180)
after_door_pose = Pose([1.9, 4.5, 0], [0, 0, 1, 0])

new_q = axis_angle_to_quaternion((0,0,1), -90)
new_ori = multiply_quaternions(new_q, robot_orientation1)

pose_corner = Pose([1.9, 2.85, 0], [0, 0, 0, 1])

after_door_ori = Pose([1.9, 4.5, 0], [0, 0, -0.7, 0.6])

# Pose in the passage between kitchen and living room
robot_orientation = Pose([3.65, 2.85, 0], [0,0,1,0])
#door_to_couch = Pose([3.65, 2.85, 0], robot_orientation)


new_q = axis_angle_to_quaternion((0,0,1), -90)
#new_ori = multiply_quaternions(new_q, robot_orientation)
door_to_couch_orientation = Pose([3.65, 2.85, 0], new_ori)

#door_to_couch_look = Pose([3.65, 2.8, 0.8], robot_orientation)





#move.pub_now(table_pose)
pose1 = Pose([1.35, 4.4, 0], [0, 0, 1, 0])
move.pub_now(after_door_pose, interrupt_bool=False)
move.pub_now(after_door_ori, interrupt_bool=False)
