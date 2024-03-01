from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from demos.pycram_receptionist_demo.utils.misc import *
from pycram.helper import axis_angle_to_quaternion
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
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

robot_orientation_couch = axis_angle_to_quaternion([0, 0, 1], 0)
pose_couch = Pose([3, 5, 0], robot_orientation_couch)

robot_orientation_from_couch = axis_angle_to_quaternion([0, 0, 1], -90)
pose_from_couch = Pose([4.2, 3.8, 0], robot_orientation_from_couch)

robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)

pose_home = Pose([3, 1.7, 0], robot_orientation)

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)

data_received = False


def talk_request_school(data: String):
    """
    callback function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: String "name drink"
    """
    global data_received
    rospy.loginfo("in callback success")
    name_drink = data.data.split(" ")
    talk_actions.name_drink_talker(name_drink)
    rospy.loginfo("nlp data:" + name_drink[0] + " " + name_drink[1])
    data_received = True


def demo_test(area):
    with real_robot:
        global data_received
        data_received = False
        print("start demo")

        # look for human
        DetectAction(technique='human', state='start').resolve().perform()
        rospy.loginfo("human detected")

        # look at guest and introduction
        giskardpy.move_head_to_human()
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(1)

        # signal to start listening
        pub_nlp.publish("start listening")

        # wait for human to say something
        rospy.sleep(5)

        # get name and drink
        rospy.Subscriber("nlp_feedback", Bool, talk_error)

        rospy.Subscriber("nlp_out", String, talk_request_school)

        while not data_received:
            rospy.sleep(0.5)

        # sleep so that toya finishes sentence
        rospy.sleep(3)

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
            rospy.loginfo("Navigating now")
            rospy.sleep(3)

            NavigateAction([pose_kitchen_to_couch]).resolve().perform()
            NavigateAction([pose_couch]).resolve().perform()

            rospy.sleep(1)
            TalkingMotion("Welcome to the living room").resolve().perform()
            TalkingMotion("take a seat").resolve().perform()

        elif area == 'from_couch':
            rospy.loginfo("Navigating now")
            rospy.sleep(3)
            NavigateAction([pose_from_couch]).resolve().perform()
            NavigateAction([pose_home]).resolve().perform()

        else:
            TalkingMotion("not navigating").resolve().perform()
            rospy.sleep(3)
            print("end")

        TalkingMotion("End of demo").resolve().perform()


# demo_test('from_couch')
demo_test('to_couch')
