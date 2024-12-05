from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
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
        ParkArmsAction([Arms.LEFT]).resolve().perform()

        json_string = get_object_info("glass")

        with open("output.json", "w") as json_file:
            json_file.write(str(json_string))

        TalkingMotion("end demo").perform()


demo(1)
