import rospy

import pycram.external_interfaces.giskard_new as giskardpy
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater

from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter


world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
#robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

#map von suturo reinbring
apartment = Object("apartment", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")

talk = TextToSpeechPublisher()


# Fallback positions for navigation
move_to_table = []

#jule
#1. was sagt nlp
# variables for communcation with nlp
def demo(step):
    global callback
    callback = False


    pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)

    # Declare Fallback Objects for demo
    muesli_box = Object("Muesli", ObjectType.BREAKFAST_CEREAL)

    while step <= 0:
        rospy.loginfo("Starting Demo")

        talk.pub_now("Starting demo")

        talk.pub_now("Say something that I can get for you.")

        while not callback:
            rospy.sleep(1)





