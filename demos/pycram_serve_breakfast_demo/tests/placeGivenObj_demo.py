import giskardpy
import rospy

from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchen.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)


# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    TalkingMotion("Starting demo").perform()
    rospy.loginfo("Starting demo")

    NavigateAction(target_locations=[Pose([1.45, 5, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    object_desig = DetectAction(technique='all').resolve().perform()
    bowl = get_bowl(object_desig)
    PlaceGivenObjAction(["Milkpackja"], ["left"],
                       [Pose([bowl.pose.position.x + 0.2, 5.8, 0.775])], ["top"]).resolve().perform()
    # PlaceGivenObjAction(["Cronybox"], ["left"], [Pose([4.86, 2, 0.88])], ["front"]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()