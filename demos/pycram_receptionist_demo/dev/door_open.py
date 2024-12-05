from geometry_msgs.msg import Vector3

from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.giskard import grasp_doorhandle, open_doorhandle
from pycram.external_interfaces.knowrob import get_object_info
from pycram.process_module import real_robot
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import ImageSwitchPublisher
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

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")

image_switch_publisher = ImageSwitchPublisher()


def demo(step: int):
    with (real_robot):
        TalkingMotion("start demo").perform()
        print(robot.joints)
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        offset = Vector3()
        offset.z = -0.01
        offset.y = -0.015
        grasp_doorhandle("iai_kitchen/iai_kitchen:arena:door_handle_inside", offset)
        MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
        open_doorhandle("iai_kitchen/iai_kitchen:arena:door_handle_inside")


demo(1)
