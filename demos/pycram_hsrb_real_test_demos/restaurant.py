import math
from datetime import time

import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped, PoseWithCovarianceStamped
from robokudo_msgs.msg import QueryAction, QueryGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from demos.pycram_hsrb_real_test_demos.utils.misc_restaurant import Restaurant

from pycram.demos.pycram_hsrb_real_test_demos.utils.NLPRestaurant import NLPRestaurant
# from pycram.demos.pycram_hsrb_real_test_demos.utils.misc_restaurant import Restaurant, monitor_func
from pycram.designators.action_designator import ParkArmsAction, DetectAction, LookAtAction, MoveTorsoAction, \
    NavigateAction
from pycram.designators.motion_designator import MoveGripperMotion, TalkingMotion
from pycram.datastructures.enums import Arms, ImageEnum
from pycram.designators.object_designator import CustomerDescription
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.local_transformer import LocalTransformer
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.datastructures.pose import Pose
from pycram.process_module import real_robot
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
import pycram.external_interfaces.giskard as giskardpy
import pycram.external_interfaces.robokudo as robokudo
import rospy
import tf
from geometry_msgs.msg import PoseStamped

from pycram.robot_description import RobotDescription
from pycram.ros_utils.viz_marker_publisher import ManualMarkerPublisher
from pycram.utilities.robocup_utils import pakerino

# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()

rospy.loginfo("Waiting for action server")
rospy.loginfo("You can start your demo now")
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)

nlp = NLPRestaurant()

###########################################################################

# Initialize global variable
global human_pose
human_pose = None
order = ('Cola', 2)


###########################################################################
#NLP Functions

##########################################################################

def monitor_found_waving():
    global human_pose
    if human_pose:
        print("Human found")
        return HumanNotFoundCondition
    if not human_pose:
        print("Human not waving?")
        return False


def transform_camera_to_x(pose, frame_x):
    """
    transforms the pose with given frame_x, orientation will be head ori and z is minus 1.3
    """
    pose.pose.position.z -= 1.3
    pose.header.frame_id = "hsrb/" + frame_x
    tPm = tf_listener.transformPose("/map", pose)
    tPm.pose.position.z = 0
    pan_pose = robot.get_link_pose("head_pan_link")
    tPm.pose.orientation = pan_pose.pose.orientation
    return tPm


def detect_waving():
    """
    Sets global human pose when a human waving was detected
    """
    talk = True
    global human_pose
    text_to_speech_publisher.pub_now("Please wave you hand. I will come to you", talk)
    human_pose = DetectAction(technique='waving', state='start').resolve().perform()


def look_around(increase: int, star_pose: PoseStamped, talk):
    """
    Function to make Toya look continuous from left to right. It stops if Toya perceives a human.
    :param: increase: The increments in which Toya should look around.
    """
    global human_pose
    tmp_x = star_pose.pose.position.x
    tmp_y = star_pose.pose.position.y
    tmp_z = star_pose.pose.position.z

    def looperino():
        global human_pose
        for x in range(-100, 100, increase):
            angle = x / 10
            look_pose = Pose([tmp_x, tmp_y, tmp_z],
                             frame="hsrb/" + RobotDescription.current_robot_description.base_link)
            look_pose_in_map = tf_listener.transformPose("/map", look_pose)
            look_point_in_map = look_pose_in_map.pose.position

            LookAtAction(
                [Pose([look_point_in_map.x + angle, look_point_in_map.y + angle,
                       look_point_in_map.z])]).resolve().perform()
            if human_pose: break

    # Executes in parallel and breaks if human_pose != None
    plan = Code(detect_waving) | Code(looperino)
    plan.perform()





def demo(step):
    # Variables for the current customer
    global customer
    customerCounter = 0
    with real_robot:
        talk = True
        start_pose = robot.get_pose()
        # Saves the original pose to be able to collect the order
        kitchenPose = start_pose
        if step <= 0:
            # So that Toya can perceive its surrounding without having its limbs in the way
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
            pakerino(config=config_for_placing)

            MoveTorsoAction([0.1]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

        if step <= 1:
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
            # Searching for a human
            look_around(20, start_pose, talk)
            # Preparing to move to detected human
            MoveTorsoAction([0]).resolve().perform()
            # Transformation of detected pose to nearest frame link
            drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
            if human_pose is not None:
                # Saving important data in customer variable
                image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
                customerCounter += 1
                customer = CustomerDescription(customerCounter, drive_pose)
            marker.publish(Pose.from_pose_stamped(drive_pose), color=[1, 1, 0, 1], name="human_waving_pose")
            move.pub_now(navpose=drive_pose)
        if step <= 2:  # Order step
             image_switch_publisher.pub_now(ImageEnum.ORDER.value)
             MoveTorsoAction([0.1]).resolve().perform()
             TalkingMotion("What do you want to order?").perform()  # NLP Placeholder
             customer.set_order("Cola",1)
             TalkingMotion(f"You want to order {customer.order[0]} in the following amount {customer.order[1]}").perform()
        if step <= 3:  # Drive back step
              image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
              TalkingMotion("I will drive back now and return with your order").perform()
              MoveTorsoAction([0]).resolve().perform()
              move.pub_now(navpose=kitchenPose)
              TalkingMotion(f"Please prepare the following order for customer {customer.id}. {customer.order[1]} {customer.order[0]}, please").perform()
        if step <= 4:
            # Present the order
             TalkingMotion(f"The customer with the ID {customer.id} wants to order {customer.order[1]} {customer.order[0]}").perform()
             # wait for the cook
             rospy.sleep(1.5)
             move.pub_now(navpose=customer.pose)
             print("demo-done")


demo(0)

