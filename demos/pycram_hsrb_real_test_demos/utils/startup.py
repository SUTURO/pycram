from pycram.datastructures.dataclasses import Color
from pycram.ros_utils.robot_state_updater import RobotStateUpdater, KitchenStateUpdater

from pycram.datastructures.enums import ObjectType

from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher


def startup():
    world = BulletWorld()
    v = VizMarkerPublisher()

    #world = BulletWorld()
    v = VizMarkerPublisher()
    kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")
    text_to_speech_publisher = TextToSpeechPublisher()
    image_switch_publisher = ImageSwitchPublisher()

    robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
    xcolor = Color(0.5, 0.5, 0.9, 1)
    robot.set_color(xcolor)

    RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
    KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
    move = PoseNavigator()

    return world, v, text_to_speech_publisher, image_switch_publisher, move, robot
