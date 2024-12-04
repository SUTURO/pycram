import math
from datetime import time

import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped, PoseWithCovarianceStamped
from robokudo_msgs.msg import QueryAction, QueryGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from demos.pycram_hsrb_real_test_demos.utils.misc_restaurant import Restaurant
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
from demos.pycram_hsrb_real_test_demos.utils.nlp_restaurant import nlp_restaurant
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

nlp = nlp_restaurant()

###########################################################################

# Initialize global variable
global human_pose
human_pose = None
order = ('Cola', 2)


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
    transforms the pose with given frame_x, orientation will be head ori and z is minu 1.3
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

    # Executes in paralel and breaks if human bool gets true
    plan = Code(detect_waving) | Code(looperino)
    plan.perform()



def detect_obstacles():
    def laser_scan_cb(msg):
        ranges = list(msg.ranges)
        print(ranges)
    laser_subscriber = rospy.Subscriber("hsrb/base_scan", LaserScan, laser_scan_cb )


def demo(step):
    global customer
    customerCounter = 0
    with real_robot:
        talk = True
        start_pose = robot.get_pose()
        TalkingMotion("start demo").perform()
        kitchenPose = start_pose
        if step <= 0:
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
            perceive_conf = {'arm_lift_joint': 0.20, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
            pakerino(config=config_for_placing)

            # todo: eign müsste sie schräg stehen weil sonst arm in weg und sie muss auf 0.5 hoch das ist nur zum gucken! nicht fahren
            MoveTorsoAction([0.1]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

        if step <= 1:
           # text_to_speech_publisher.pub_now("Starting Restaurant Demo.", talk)
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
            look_around(5, start_pose, talk)
            MoveTorsoAction([0]).resolve().perform()
            drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
            if human_pose is not None:
                image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
                customerCounter += 1
                customer = CustomerDescription(customerCounter, human_pose)
            marker.publish(Pose.from_pose_stamped(drive_pose), color=[1, 1, 0, 1], name="human_waving_pose")
            move.pub_now(navpose=drive_pose)
            detect_obstacles()
            if robot.get_pose() != drive_pose:
                try:
                    move.pub_now(navpose=drive_pose)
                except EnvironmentError:
                    print("Position of customer is not reachable. Please re-position yourself")
            print(customer.pose)
            print(customer.id)
        # if step <= 2:  # Order step
        #     image_switch_publisher.pub_now(ImageEnum.ORDER.value)
        #     MoveTorsoAction([0.1]).resolve().perform()
        #     TalkingMotion("What do you want to order?").perform()  # NLP Placeholder
        #     customer.set_order(order)
        # if step <= 3:  # Drive back step
        #     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
        #     TalkingMotion("I will drive back now and return with your order").perform()
        #     MoveTorsoAction([0]).resolve().perform()
        #     move.pub_now(navpose=kitchenPose)
        #     print("I am back")
        # if step <= 4:
        #     # Present the order
        #     TalkingMotion(
        #         f"The customer with the ID {customer.id} wants to order {customer.order[1]} {customer.order[0]}")
        #     # wait for the cook
        #     rospy.sleep(3)
        #     move.pub_now(navpose=customer.pose)
        #     print("demo-done")


demo(1)

# while not success:
#     try:
#      look_around(1, start_pose, talk)

# pose = Pose([new_human_p.point.x, new_human_p.point.y, 0])
#
# print(type(new_human_p))
# new_human_p = DetectAction(technique='waving', state='stop').resolve().perform()
#
# if pose is not None:
#     human_bool = True
# #human_pose_inM = Pose[(new_human_p.point.x, new_human_p.point.y, new_human_p.z)]
# print(pose)

#                    MoveTorsoAction([0]).resolve().perform()
# tf_pose_bl = tf_listener.transformPose(
#    "hsrb/" + RobotDescription.current_robot_description.base_link, pose)
# NavigateAction([Pose([tf_pose_bl.pose.position.x - 0.2, tf_pose_bl.pose.position.y - 0.2, 0])])


# print("Human Point", new_human_p)
# print("Human Point in Map", human_point_in_map)
# print("Human Pose in Map", human_pose_in_map)
# print("Human Pose in Bl", human_pose_in_bl)
# print("Toyas Pose in frame", restaurant.toya_pose)

# if human_pose_in_bl is not None:
#   human_bool = True
#   NavigateAction(target_locations=[Pose([human_pose_in_map.position.x-0.25, human_pose_in_map.position.y-0.25, 0])]).resolve().perform()
#
# except Exception as e:
#     print(e)
#         try:
#             MoveTorsoAction([0]).resolve().perform() # Damit Toya eingefahren ist bevor sie losfährt
#             plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
#             plan.perform()
#         except SensorMonitoringCondition:
#             text_to_speech_publisher.pub_now("Finding my way. Please wait.", talk)
#             image_switch_publisher.pub_now(ImageEnum.HI.value)
#
# if step <= 2:
#     text_to_speech_publisher.pub_now("Driving.", talk)
#     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
#     try:
#         plan = Code(lambda: NavigateAction([human_pose_in_map.pose.position.x-0.2, human_pose_in_map.pose.position.y-0.2, 0]).resolve().perform()) >> Monitor(monitor_func)
#         plan.perform()
#     except SensorMonitoringCondition:
#         print("ABBRUCH ALARM ALARM")
#         #TODO schauen, ob move interrupt vielleicht schon in einem Designator festgelegt worden ist
#         move.interrupt()

#          try:
#                 plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
#                 plan.perform()
#             except SensorMonitoringCondition:
#                 text_to_speech_publisher.pub_now("Finding my way. Please wait.", talk)
#                 image_switch_publisher.pub_now(ImageEnum.HI.value)
#
#
#             # human_poseTm = transform_pose(human_pose, "head_rgbd_sensor_rgb_frame", "map")
# human_p = human_poseTm
# human_p.pose.position.z = 0
# human_p.pose.orientation.x = 0
# human_p.pose.orientation.y = 0
# human_p.pose.orientation.z = 0
# #             restaurant.human_pose = None
#             human_pose = robokudo.query_waving_human()
#             human_poseTm = transform_pose(human_pose, "head_rgbd_sensor_rgb_frame", "map")
#             human_p = human_poseTm
#             human_p.pose.position.z = 0
#             human_p.pose.orientation.x = 0
#             human_p.pose.orientation.y = 0
#             human_p.pose.orientation.z = 0
#             human_p.pose.orientation.w = 1
#             human_p.pose.orientation.w = 1
#
#             if human_p:
#                 success = True
#                 restaurant.human_pose = human_p
#                 # print(human_p.pose.position.x)


#                except Exception as e:
#                   print("An error occurred from perception:", e)
# if step <= 2:
#     text_to_speech_publisher.pub_now("Driving", talk)
#     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
#     world.current_bullet_world.add_vis_axis(Pose.from_pose_stamped(human_p))
#     #print(restaurant.distance())
#     marker.publish(Pose.from_pose_stamped(human_p))
#     world.current_bullet_world.add_vis_axis(Pose.from_pose_stamped(human_p))
#     try:
#         plan = Code(lambda: move.pub_now(human_p)) >> Monitor(monitor_func)
#         plan.perform()
#     except SensorMonitoringCondition:
#         print("ABBRUCH")
#         move.interrupt()
#     # robot_pose_to_human = Pose.from_pose_stamped(calc_distance(human_p, 0.5))

# world.current_bullet_world.add_vis_axis(robot_pose_to_human)
#     move.query_pose_nav(robot_pose_to_human)
#     image_switch_publisher.pub_now(ImageEnum.ORDER.value)
#     text_to_speech_publisher.pub_now("Hi what do you want to have.", talk)
#     # nlp stuff
# if step <= 3:
#     text_to_speech_publisher.pub_now("Driving", talk)
#     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
#     move.query_pose_nav(start_pose)
#     # nlp stuff insert here
#     image_switch_publisher.pub_now(ImageEnum.HANDOVER.value)
#     text_to_speech_publisher.pub_now("The guest wants: apple, milk", talk)
#
# if step <= 4:
#     text_to_speech_publisher.pub_now("Driving", talk)
#     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
#     move.query_pose_nav(robot_pose_to_human)
#     # nlp stuff insert here
#     image_switch_publisher.pub_now(ImageEnum.HANDOVER.value)
#     text_to_speech_publisher.pub_now("Take out your order", talk)
#
# if step <= 5:
#     text_to_speech_publisher.pub_now("Driving", talk)
#     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
#     move.query_pose_nav(start_pose)
#     demo(step=1)


# human_pose = detect_waving(talk)
# if not human_pose:
#  # rospy.sleep(1)

# for w in range(10, -10, -increase):
#     human_pose = detect_waving(talk)
#     if not human_pose:
#         angle = w / 10
#         look_pose = Pose([tmp_x, tmp_y, tmp_z],
#                          frame="hsrb/" + RobotDescription.current_robot_description.base_link)
#         look_pose_in_map = tf_listener.transformPose("/map", look_pose)
#         look_point_in_map = look_pose_in_map.pose.position
#
#         LookAtAction(
#             [Pose([look_point_in_map.x + angle, look_point_in_map.y + angle,
#                    look_point_in_map.z])]).resolve().perform()
#         # rospy.sleep(1)
#

#
# def monitor_func_human():
#     der = fts.get_last_value()
#     if abs(der.wrench.force.x) > 10.30:
#         print("sensor")
#      return SensorMonitoringCondition
#     if not human.human_pose.get_value():
#         print("human")
#         return HumanNotFoundCondition
#     return False
#
#
# def monitor_func():
#     der = fts.get_last_value()
#     if abs(der.wrench.force.x) > 10.30:
#         return SensorMonitoringCondition
#     return False
#
#
# def callback(point):
#     global human_bool
#     human_bool = True
#     print("Human detected")

# class Human:
#     """
#     Class that represents humans. This class does not spawn a human in a simulation.
#     """
#
#     def __init__(self):
#         self.human_pose = Fluent()
#
#         self.last_msg_time = time.time()
#
#         self.threshold = 5.0  # seconds
#
#         # Subscriber to the human pose topic
#         self.human_pose_sub = rospy.Subscriber("/cml_human_pose", PointStamped, self.human_pose_cb)
#
#         # Timer to check for no message
#         self.timer = rospy.Timer(rospy.Duration(1), self.check_for_no_message)
#
#     def check_for_no_message(self, event):
#         current_time = time.time()
#         if (current_time - self.last_msg_time) > self.threshold:
#             # rospy.loginfo("No messages received for %s seconds", self.threshold)
#             self.human_pose.set_value(False)
#
#     def human_pose_cb(self, HumanPoseMsg):
#         """
#         Callback function for human_pose Subscriber.
#         Sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic.
#         :param HumanPoseMsg: received message
#         """
#         self.last_msg_time = time.time()
#
#         if HumanPoseMsg:
#             self.human_pose.set_value(True)
#         else:
#             self.human_pose.set_value(False)
#
#
# class Restaurant:
#     def __init__(self):
#         self.toya_pose = Fluent()
#         self.human_pose = None
#         self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)
#         self.robot = robot
#         self.rospy = rospy
#
#     def toya_pose_cb(self, msg):
#         # print("updating")
#         self.toya_pose.set_value(robot.get_pose())
#         self.rospy.sleep(0.5)
#
#     def set_human_pose(self, pose: Pose):
#         self.human_pose = pose
#
#     def distance(self):
#         print("toya pose:" + str(self.toya_pose.get_value().pose))
#         if self.human_pose:
#             dis = math.sqrt((self.human_pose.pose.position.x - self.toya_pose.get_value().pose.position.x) ** 2 +
#                             (self.human_pose.pose.position.y - self.toya_pose.get_value().pose.position.y) ** 2)
#             print("dis: " + str(dis))
#             return dis
#         else:
#             self.rospy.logerr("Cant calculate distance, no human pose found")
#
#
# def monitor_func(restaurant: Restaurant):
#     if restaurant.distance() < 1:
#         return SensorMonitoringCondition
#     return False
#

# restaurant = Restaurant()

# def human_pose_cb(data):
#     query_result = data
#     return data
