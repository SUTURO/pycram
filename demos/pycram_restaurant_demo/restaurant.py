import math
import threading
import time
from collections import OrderedDict
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv_bridge

import pycram
import tf.transformations
from tf.transformations import *
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from demos.pycram_restaurant_demo.utils.stuck_detector import StuckDetector
from pycram.designators.motion_designator import *
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from demos.pycram_restaurant_demo.utils.nlp_restaurant import nlp_restaurant
from pycram.datastructures.enums import Arms, ImageEnum
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, DetectAction, LookAtAction, MoveTorsoAction, \
    NavigateAction
from pycram.designators.motion_designator import TalkingMotion, MoveJointsMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.external_interfaces.robokudo import get_used_annotator_list
from pycram.failures import HumanNotFoundCondition
from pycram.language import Code, Monitor
from pycram.process_module import real_robot
from pycram.robot_description import RobotDescription
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor

from pycram.utilities.robocup_utils import pakerino, TextToImagePublisher, ImageSendPublisher

# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()
text_to_img_publisher = TextToImagePublisher()
rospy.loginfo("Waiting for action server")
rospy.loginfo("You can start your demo now")
response = [None, None]
vel_response = [None]
bridge = CvBridge()
stopped = False
# isp = ImageSendPublisher()
#
callback = False
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)
nlp = nlp_restaurant()
odom_response = None
# stuck_detector = StuckDetector()
pose_dict = OrderedDict()
moving = False
stuck = False
###########################################################################

# Initialize global variable
global human_pose
last_position = None
stuck_time = rospy.Time.now()
human_pose = None
timeout = 10
customers = list()
global customerCounter
customerCounter = 0
fts = ForceTorqueSensor(robot_name='hsrb')
# Pose required because of multiple customers
# kitchen_pose = Pose([3.66, 2.09, 0.75], [0,0,-0.7, 0.64])
global kitchen_pose




def transform_camera_to_x(pose, frame_x):
    """
    transforms the pose with given frame_x, orientation will be head ori and z is minus 1.3
    """
    pose.pose.position.z -= 1.3

    pose.header.frame_id = "hsrb/" + frame_x
    tPm = tf_listener.transform_pose(pose=pose, target_frame="/map")
    tPm.pose.position.z = 0
    pan_pose = robot.get_link_pose("head_pan_link")
    pan_pose.header.frame_id = "/map"
    tPm.pose.orientation = pan_pose.pose.orientation

    return tPm


def look_around(increase: float, star_pose: PoseStamped, talk):
    """
    Function to make Toya look continuous from left to right. It stops if Toya perceives a human.
    :param: increase: The increments in which Toya should look around.
    """

    global human_pose
    human_pose = None
    tmp_x = star_pose.pose.position.x
    tmp_y = star_pose.pose.position.y
    tmp_z = star_pose.pose.position.z
    x = -0.5
    while x <= 1:

        MoveJointsMotion(["head_pan_joint"], [x]).perform()
        try:
            human_pose = DetectAction(technique='waving', state='start').resolve().perform()
        except pycram.failures.PerceptionObjectNotFound:
            print("oh no, no waving human was found")
        if human_pose:
            break

        x += increase
        if x == 1:
            x = -0.5


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor exception")
        return SensorMonitoringCondition

    return False


def confirmation():
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """
    try:

        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()

        image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        print("done")
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        rospy.sleep(2)
        return


def change_orientation(startPose: Pose):
    """
    Method to move the base of Toya in an 180° Angle.
    """
    quat_o = (startPose.pose.orientation.x, startPose.pose.orientation.y, startPose.pose.orientation.z,
              startPose.pose.orientation.w)
    print(quat_o)
    angle_to_add = np.pi
    q_add = tf.transformations.quaternion_from_euler(0, 0, angle_to_add)
    q_add_new = (q_add[0], q_add[1], q_add[2], q_add[3])

    q_new = tf.transformations.quaternion_multiply(quat_o, q_add_new)
    new_angle = (q_new[0], q_new[1], q_new[2], q_new[3])
    newPose = Pose([startPose.pose.position.x, startPose.pose.position.y, startPose.pose.position.z],
                   [new_angle[0], new_angle[1], new_angle[2], new_angle[3]])
    return newPose




def laser_callback(msg):
    global stuck
    min_dist = np.min(msg.ranges)
    if moving:
        if min_dist < 0.3:
            rospy.logwarn("Obstacle too close")
            if stuck:
                TalkingMotion("There is an obstacle in my way.").perform()
                rospy.sleep(2)

def cmd_vel_callback(msg):
    global stuck
    if moving:
        if abs(msg.linear.x) < 0.00 and abs(msg.angular.z) < 0.00:
            rospy.logwarn("No Movement detected")
            stuck = True


def move_towards(goal: Pose):
    global moving
    if goal is not None:
        moving = True
        move.pub_now(navpose=goal)

def set_pose_in_front(goalPose: Pose, robotPose: Pose, dist : float):
    rotation_matrix = quaternion_matrix([goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])

    forward_vector = rotation_matrix[:3, 0]
    distance = dist
    new_pos = np.array([goalPose.pose.position.x, goalPose.pose.position.y, goalPose.pose.position.z]) - distance * forward_vector
    adjusted_pose = Pose(position=[new_pos[0], new_pos[1], new_pos[2]], orientation=[goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])
    print(adjusted_pose)
    return adjusted_pose

def demo(step: int):
   # rospy.Subscriber("/hsrb/odom", Odometry, cmd_vel_callback)
   # rospy.Subscriber("/hsrb/base_scan", LaserScan, laser_callback)
    global customer, customerCounter, kitchen_pose, human_pose

    with real_robot:

        talk = True
        start_pose = robot.get_pose()
        kitchen_pose = start_pose
        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        rospy.sleep(2)

        if len(customers) == 0:
             TalkingMotion("start restaurant demo").perform()
             rospy.sleep(2)
             TalkingMotion("Please push down my gripper to start the demo ").perform()
             image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)

             try:
                 plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                 plan.perform()
             except SensorMonitoringCondition:
                 image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 0:
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
            pakerino(config=config_for_placing)
            MoveTorsoAction([0.2]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion("Please wave, so that I can perceive you").perform()

        if step <= 1:
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
            annotator = get_used_annotator_list(Demos.RESTAURANT, as_topic_names=False)

            isp = ImageSendPublisher(sub_topic=annotator[0])
            isp.activate_subscriber()
            rospy.sleep(2)
            look_around(0.5, start_pose, talk)
            MoveTorsoAction([0]).resolve().perform()

            if human_pose is not None:
                # Changes image to the results of perception

                image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
                rospy.sleep(2)
                drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
                print(drive_pose)
                adjusted_drive_pose = set_pose_in_front(drive_pose, robot.get_pose(), 0.6)

                customerCounter += 1

                customer = CustomerDescription(customerCounter, adjusted_drive_pose)
                customers.append(customer)

            marker.publish(Pose.from_pose_stamped(adjusted_drive_pose), color=[1, 1, 0, 1], name="human_waving_pose")
            rospy.sleep(2.5)
           # move_towards(drive_pose)
            move.pub_now(navpose=adjusted_drive_pose)
            #plan = Code(move.pub_now(navpose=drive_pose) | lol(drive_pose))
            #plan.perform()


            rospy.sleep(1)

        if step <= 2:  # Order step
            image_switch_publisher.pub_now(ImageEnum.ORDER.value)
            MoveTorsoAction([0.1]).resolve().perform()
            LookAtAction([Pose([robot.pose.position.x, robot.pose.position.y, 0.8])])
            rospy.sleep(1)
            Timmi = CustomerDescription(id=1, pose=start_pose)
            # customer = Timmi
            nlp.get_order(customer=customer)
            print(customer.order)
            rospy.sleep(2)
            if customer.order is not None:
                nlp.confirm_order(customer=customer, order=customer.order)
        if step <= 3:  # Drive back step
            TalkingMotion("I will drive back now and return with your order").perform()
            rospy.sleep(2.5)

            change_o = change_orientation(robot.get_pose())
            NavigateAction([change_o]).resolve().perform()
            rospy.sleep(2)
            image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
            MoveTorsoAction([0]).resolve().perform()
            rospy.sleep(2)

            order_kitchen_pose = change_orientation(kitchen_pose)

            move.pub_now(navpose=order_kitchen_pose)

            rospy.sleep(2.5)
            print("order", customer.order)
            if len(customer.order) == 1:
                TalkingMotion(f"Please prepare the order {customer.order[0][1]} {customer.order[0][0]}").perform()
                text_to_img_publisher.pub_now(f"The order: {customer[0][1]} {customer[0][0]}")
                rospy.sleep(2)
                image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
            elif len(customer.order) >= 2:
                TalkingMotion("Please prepare the following order").perform()
                txt_order = "The order: "
                for n in customer.order:
                    TalkingMotion(f"{n[1]}{n[0]}").perform()
                    txt_order += f" {n[1]} {n[0]} " + "\n"
                text_to_img_publisher.pub_now(txt_order)
                rospy.sleep(2)
                image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
            TalkingMotion("Please put the order in the tray in my gripper").perform()
            rospy.sleep(2)
            TalkingMotion(f"Please push down my gripper, if the order is prepared and my display changed").perform()
            rospy.sleep(1)
            confirmation()

            rospy.sleep(3)
            TalkingMotion("I will bring the order to the customer now").perform()
        if step <= 4:
            kitchen_to_cust_orientation = change_orientation(order_kitchen_pose)
            NavigateAction([kitchen_to_cust_orientation]).resolve().perform()
            rospy.sleep(2.5)
            move.pub_now(navpose=customer.pose)
            rospy.sleep(2.5)
            TalkingMotion("Here is your order. Please take it out of my tray").perform()
            rospy.sleep(2)
            TalkingMotion("Please push down my gripper, if you took your order and my display changed").perform()
            rospy.sleep(2)
            confirmation()
            TalkingMotion("I will drive back now to search for new customers").perform()
            rospy.sleep(1)
            cust_to_kitchen_orentation = change_orientation(robot.get_pose())
            NavigateAction([cust_to_kitchen_orentation]).resolve().perform()
            move.pub_now(navpose=kitchen_pose)
            while len(customers) <= 2:
                print(len(customers))
                demo(0)


demo(0)
