import rospy

# from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl, sort_objects, try_pick_up
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot, semi_real_robot

from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
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

# robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_1.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)


# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    TalkingMotion("Starting demo").perform()
    angle = 0
    angle = 0
    while angle <= 1:
        print(robot.get_pose().pose.position)
        print(robot.get_pose().pose.orientation)
        # print(type(robot.get_pose()))
        pose_test = Pose(position=[3.4, 1.2, 1])
        LookAtAction([pose_test]).resolve().perform()
        # LookAtAction([Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y, 1])]).resolve().perform()
        angle += 0.5
