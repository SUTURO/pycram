import rospy
from geometry_msgs.msg import PoseStamped
from robokudo_msgs.msg import QueryActionGoal
from pycram.designators.action_designator import DetectAction, LookAtAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion, MoveMotion
from pycram.external_interfaces import robokudo
from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot, semi_real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
import talk_actions

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
milk = Object("Milkpack", "milk", "milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])

giskardpy.init_giskard_interface()


# giskardpy.sync_worlds()
# RobotStateUpdater("/tf", "/joint_states")


class HumanDescription:

    def __init__(self, name, fav_drink, shirt_color, gender):
        self.name = name
        self.fav_drink = fav_drink
        self.shirt_color = shirt_color
        self.gender = gender
        # TODO: coordinate with Perception on what is easy to implement
        # characteristics to consider: height, hair color, and age.


def talk_request(data: String):
    """
    callback function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: String "name drink"
    """

    name_drink = data.data.split(" ")
    talk_actions.name_drink_talker(name_drink)
    rospy.loginfo("nlp data:" + name_drink[0] + " " + name_drink[1])


def talk_error(data):
    """
    callback function if no name/drink was heard
    """

    error_msgs = "i could not hear you, please repeat"
    TalkingMotion(error_msgs).resolve().perform()
    pub_nlp.publish("start listening")


with real_robot:
    # Perception
    perceived_object_dict = DetectAction(BelieveObject(types=[milk.type]), technique='human').resolve().perform()
    while perceived_object_dict[0] is None:
        TalkingMotion("Please step in front of me")
        rospy.sleep(5)

    rospy.loginfo("human detected")
    pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)

    giskardpy.move_head_to_human()

    TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()

    # reicht sleep 1?
    rospy.sleep(1)

    # signal to start listening
    pub_nlp.publish("start listening")

    # TODO: knowlede interface erweitern
    # Idee: jeder Mensch braucht ID, diese kann ich dann hier abfragen
    while knowrob_call not filled:
        if knowrob_call.name:
            TalkingMotion("please repeat your favorite drink")

        if knowrob_call.drink:
            TalkingMotion("please repeat your name")

        # failure handling if receptionist intend was not understood
        rospy.Subscriber("nlp_feedback", Bool, talk_error)

    # TODO: Position der Couch herausfinden
    giskardpy.stop_looking()
    NavigateAction([Pose([5, 3.3, 0.8], [0, 0, 1, 1])]).resolve().perform()

    DetectAction(BelieveObject(types=[milk.type]), technique='human').resolve().perform()

    #Personen einander vorstellen









