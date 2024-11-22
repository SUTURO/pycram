import time

import geometry_msgs.msg

from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, real_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.external_interfaces import giskard_new as giskard
world = BulletWorld()
viz = VizMarkerPublisher()
robot = Object("hsrb", "robot", "hsrb.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", "environment", "suturo_lab_version_15.urdf")
giskard.init_giskard_interface()
robot_desig = BelieveObject(names=["hsrb"])
apartment_desig = BelieveObject(names=["apartment"])


pick_pose = Pose([2.7, 2.15, 1])
nav_pose = Pose([4, 3.8, 0])
nav_pose1 = Pose([3.9, 3.8, 0], [0, 0, 1, 0])
# new_pose = geometry_msgs.msg.PoseStamped()

with real_robot:
    giskard.achieve_sequence_te(nav_pose1)