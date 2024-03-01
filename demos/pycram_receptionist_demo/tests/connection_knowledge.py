import rospy
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
from std_msgs.msg import String
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

robot_orientation_couch = axis_angle_to_quaternion([0, 0, 1], 0)
pose_couch = Pose([3, 5, 0], robot_orientation_couch)

robot_orientation_from_couch = axis_angle_to_quaternion([0, 0, 1], -90)
pose_from_couch = Pose([4.2, 3.8, 0], robot_orientation_from_couch)

robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)

pose_home = Pose([3, 1.7, 0], robot_orientation)

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def demo_test():
    with real_robot:
        guest1 = HumanDescription("guest1")

        rospy.loginfo("human detected")

        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()

        # signal to start listening
        rospy.sleep(1)
        pub_nlp.publish("start listening")
        rospy.sleep(5)
        #asyncron??

        guest_data = get_guest_info("3.0")
        print("guest data: " + str(guest_data))
        while guest_data == "No name saved under this ID!":
            talk_error("no name")
            guest_data = get_guest_info("1.0")
            rospy.sleep(3)

        talk_request(guest_data)
        TalkingMotion("End of Knowledge Test").resolve.perform()


def get_guest_info_old(id):
    """
    function that uses Knowledge Service to get Name and drink from new guest via ID
    """

    #TODO: Service für Name und Getränk mit ID einbauen, wenn dieser fertig
    rospy.wait_for_service('name_server')
    print("debug")
    try:
        #TODO: Service class herausfinden
        print("in try")
        info_service = rospy.ServiceProxy('name_server', String) #isKnown
        print("serverProxy")
        guest_data = info_service(id) #guest_data = List der Form ["name", "drink"]
        print("end")
        return guest_data
    except rospy.ServiceException as e:
        print("Service call failed")



def client_test(data):
    try:
        zahl = int(data)
        ergebnis = 10 / zahl
        print("Ergebnis:", ergebnis)
        return "klappt"

    except ZeroDivisionError:
        print("Fehler: Division durch Null ist nicht erlaubt.")



if __name__ == '__main__':
    demo_test()
