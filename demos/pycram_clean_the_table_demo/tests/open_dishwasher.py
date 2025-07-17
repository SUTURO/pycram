import rospy

from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl, sort_objects, try_pick_up, get_free_spaces
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces import giskard
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
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

#robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)


# name of the dishwasher handle and dishwasher door
handle_name = "iai_kitchen/sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"
hinge_name = 'sink_area_dish_washer_door_joint'
dishwasher_main_name = "sink_area_dish_washer_main"

# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    # ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
    NavigateAction([Pose([2.8, -2.1, 0], [0, 0, -1, 1])]).resolve().perform()
    MoveJointsMotion(["wrist_roll_joint"], [-1.5]).perform()
    MoveJointsMotion(["arm_roll_joint"], [0]).perform()
    TalkingMotion("I will open the dishwasher now").perform()
    OpenDishwasherAction(handle_name, hinge_name, door_name, [Arms.LEFT]).resolve().perform()
    # giskard.open_dishwasher(handle_name, 'sink_area_dish_washer_door_joint', door_name)
