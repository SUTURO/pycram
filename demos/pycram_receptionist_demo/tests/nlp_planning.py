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

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def test():
    with real_robot:
        print("start demo")
        #TalkingMotion("Hello we will test NLP now").resolve().perform()
        #rospy.sleep(1)

        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(6)
        pub_nlp.publish("start listening")

        # failure handling
        rospy.loginfo("before Subscriber")

        # failure handling
        rospy.Subscriber("nlp_feedback", Bool, talk_error)

        # receives name and drink via topic
        rospy.Subscriber("nlp_out", String, talk_request_nlp)

        while True:
            print("Hello")


if __name__ == '__main__':
    test()