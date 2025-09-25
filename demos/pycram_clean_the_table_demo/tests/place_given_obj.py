import rospy

from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl, sort_objects, try_pick_up, get_free_spaces
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

# Initialize the Bullet world for simulation
world = BulletWorld(WorldMode.GUI)

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


# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    dishwasher_left = Pose([3.753, -2.35, 0], [0, 0, 1, 0])
    dishwasher_right = Pose([1.93, -2.35, 0], [0, 0, 0, 1])
    dishwasher_middle = Pose([2.95, -1.85, 0], [0, 0, -1, 1])
    # dishwasher_middle = Pose([3.15, -1.85, 0], [0, 0, -1, 1])
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction([dishwasher_middle]).resolve().perform()
    # if x_pos >= 2.9:
    # NavigateAction([dishwasher_left]).resolve().perform()

    TalkingMotion("Placing").perform()
    grasp = Grasp.FRONT

    MoveTorsoAction([0.2]).resolve().perform()
    PlaceGivenObjectAction(["Metalplate"],  [Arms.LEFT], [Pose([2.82, -2.65, 0.52])],
                           [Grasp.FRONT], [False], False).resolve().perform()

    # NavigateAction(target_locations=[Pose([1.4, 3.9, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    # PouringAction([Pose([3.6, 4.9, 0.72])], [Arms.LEFT], ["right"], [115])

    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveTorsoAction([0.0]).resolve().perform()
    NavigateAction([Pose(robot.get_pose().pose.position, [0, 0, 0.7, 0.7])]).resolve().perform()
    NavigateAction([Pose([2.3, 1.5, 0],[0, 0, 0.7, 0.7])]).resolve().perform()
