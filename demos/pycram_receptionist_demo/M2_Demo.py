import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.fluent import Fluent
from demos.pycram_receptionist_demo.utils.misc import *
from pycram.helper import axis_angle_to_quaternion
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.external_interfaces.knowrob import get_guest_info
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from demos.pycram_receptionist_demo.deprecated import talk_actions
import pycram.external_interfaces.navigate as moveBase

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# carefull that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "kitchen.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")

robot_orientation_couch = axis_angle_to_quaternion([0, 0, 1], 180)
pose_couch = Pose([3, 5, 0])

robot_orientation_from_couch = axis_angle_to_quaternion([0, 0, 1], -90)
pose_from_couch = Pose([4.2, 3.8, 0], robot_orientation_from_couch)

robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)

pose_home = Pose([3, 1.7, 0], robot_orientation)

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def demo_ms2(area):

    with real_robot:

        # declare variables for humans
        host = HumanDescription("Yannis", fav_drink="Tea")
        guest1 = HumanDescription("guest1")

        # look for human
        # TODO: only continue when human stands for longer than 3s in front of robot
        DetectAction(technique='human', state='start').resolve().perform()
        rospy.loginfo("human detected")

        # look at guest and introduce
        giskardpy.move_head_to_human()
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(1)

        # signal to start listening
        pub_nlp.publish("start listening")

        # wait for human to say something
        rospy.sleep(15)

        # guest_data format is = ['person infos: "name', 'drink"']
        guest_data = get_guest_info("1.0")
        while guest_data == ['person_infos: "No name saved under this ID!"']:
            talk_error("no name")
            rospy.sleep(13)
            guest_data = get_guest_info("1.0")

        # set heard name and drink of guest
        guest1.set_name(guest_data[0][13:])
        guest1.set_drink(guest_data[1])
        talk_request(guest_data)
        rospy.sleep(1)

        # lead human to living room
        TalkingMotion("i will show you the living room now").resolve().perform()
        rospy.sleep(1)
        TalkingMotion("please step out of the way and follow me").resolve().perform()
        rospy.loginfo("stop looking now")
        giskardpy.stop_looking()

        # stop perceiving human
        rospy.loginfo("stop detecting")
        DetectAction(technique='human', state='stop').resolve().perform()

        if area == 'to_couch':
            # wait for human to step out of way
            rospy.sleep(5)
            NavigateAction([pose_kitchen_to_couch]).resolve().perform()
            NavigateAction([pose_couch]).resolve().perform()
            TalkingMotion("Welcome to the living room").resolve().perform()
            rospy.sleep(1)
        elif area == 'from_couch':
            rospy.loginfo("Navigating now")
            rospy.sleep(5)
            NavigateAction([pose_from_couch]).resolve().perform()
            NavigateAction([pose_home]).resolve().perform()
        else:
            rospy.loginfo("in else")
            TalkingMotion("not navigating").resolve().perform()
            rospy.sleep(1)

        introduce(guest1.name, guest1.fav_drink, host.name, host.fav_drink)


# demo_test('from_couch')
demo_ms2('to_couch')
