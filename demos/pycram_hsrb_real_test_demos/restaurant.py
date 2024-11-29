import math
from datetime import time

import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped, PoseWithCovarianceStamped
from robokudo_msgs.msg import QueryAction, QueryGoal

from demos.pycram_hsrb_real_test_demos.utils.misc_restaurant import Restaurant
# from pycram.demos.pycram_hsrb_real_test_demos.utils.misc_restaurant import Restaurant, monitor_func
from pycram.designators.action_designator import ParkArmsAction, DetectAction, LookAtAction, MoveTorsoAction, \
    NavigateAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.datastructures.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.local_transformer import LocalTransformer
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.datastructures.pose import Pose
from pycram.process_module import real_robot
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
# import pycram.external_interfaces.giskard as giskardpy
import pycram.external_interfaces.robokudo as robokudo

import rospy
import tf
from geometry_msgs.msg import PoseStamped

from pycram.robot_description import RobotDescription
from pycram.ros_utils.viz_marker_publisher import ManualMarkerPublisher
from pycram.utilities.robocup_utils import pakerino

# Initialize the necessary components
world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()

# giskardpy.init_giskard_interface()
# robokudo.init_robokudo_interface()
marker = ManualMarkerPublisher()
listener = tf.TransformListener()

# Initialize global variable
global human_bool
human_bool = False


def monitor_func_human(humanPoint: PointStamped, robotPoint: PoseStamped):
    restaurant.set_human_pose(humanPoint)
    tmp_dist = restaurant.distance()
    if tmp_dist < 0.4:
        print("min dist reached")
        return SensorMonitoringCondition
    if humanPoint.point is None:
        print("Human went missing lol. Where did he go?")
        return False


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

class Human:
    """
    Class that represents humans. This class does not spawn a human in a simulation.
    """

    def __init__(self):
        self.human_pose = Fluent()

        self.last_msg_time = time.time()

        self.threshold = 5.0  # seconds

        # Subscriber to the human pose topic
        self.human_pose_sub = rospy.Subscriber("/cml_human_pose", PointStamped, self.human_pose_cb)

        # Timer to check for no message
        self.timer = rospy.Timer(rospy.Duration(1), self.check_for_no_message)

    def check_for_no_message(self, event):
        current_time = time.time()
        if (current_time - self.last_msg_time) > self.threshold:
            # rospy.loginfo("No messages received for %s seconds", self.threshold)
            self.human_pose.set_value(False)

    def human_pose_cb(self, HumanPoseMsg):
        """
        Callback function for human_pose Subscriber.
        Sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic.
        :param HumanPoseMsg: received message
        """
        self.last_msg_time = time.time()

        if HumanPoseMsg:
            self.human_pose.set_value(True)
        else:
            self.human_pose.set_value(False)


class Restaurant:
    def __init__(self):
        self.toya_pose = Fluent()
        self.human_pose = None
        self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)
        self.robot = robot
        self.rospy = rospy

    def toya_pose_cb(self, msg):
        # print("updating")
        self.toya_pose.set_value(robot.get_pose())
        self.rospy.sleep(0.5)

    def set_human_pose(self, pose: PoseWithCovarianceStamped):
        self.human_pose = pose

    def distance(self):
        print("toya pose:" + str(self.toya_pose.get_value().pose))
        if self.human_pose:
            dis = math.sqrt((self.human_pose.pose.position.x - self.toya_pose.get_value().pose.position.x) ** 2 +
                            (self.human_pose.pose.position.y - self.toya_pose.get_value().pose.position.y) ** 2)
            print("dis: " + str(dis))
            return dis
        else:
            self.rospy.logerr("Cant calculate distance, no human pose found")


def monitor_func(restaurant: Restaurant):
    if restaurant.distance() < 1:
        return SensorMonitoringCondition
    return False


restaurant = Restaurant()


def transform_pose(input_pose, from_frame, to_frame):
    """
    Transforms a pose from 'from_frame' to 'to_frame'.
    """
    try:
        # Ensure that the transform is available
        now = rospy.Time.now()
        listener.waitForTransform(to_frame, from_frame, now, rospy.Duration(4.0))

        # Perform the transform
        transformed_pose = listener.transformPose(to_frame, input_pose)
        return transformed_pose

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo("Transform error: %s", e)
        return None


rospy.loginfo("Waiting for action server")
rospy.loginfo("You can start your demo now")
tf_listener = LocalTransformer()


def human_pose_cb(data):
    query_result = data
    return data

def look_around(increase: int, star_pose: PoseStamped):
    """
    Function to make Toya look continuous from left to right. It stops if Toya perceives a human.
    :param: increase: The increments in which Toya should look around.
    """
    tmp_x = star_pose.pose.position.x
    tmp_y = star_pose.pose.position.y
    tmp_z = star_pose.pose.position.z
    for x in range(-10, 10, increase):
        angle = x / 10
        look_pose = Pose([tmp_x, tmp_y, tmp_z], frame="hsrb/" + RobotDescription.current_robot_description.base_link)
        look_pose_in_map = tf_listener.transformPose("/map", look_pose)
        look_point_in_map = look_pose_in_map.pose.position

        LookAtAction(
            [Pose([look_point_in_map.x + angle, look_point_in_map.y + angle, look_point_in_map.z])]).resolve().perform()
        rospy.sleep(1)
        new_human_p = DetectAction(technique='human', state="start").resolve().perform()
        if new_human_p is not None:
            tmp_human_pose = new_human_p
            new_human_p = DetectAction(technique='human', state="stop").resolve().perform()
            print(new_human_p)
            return tmp_human_pose
        #if human_bool:
         #   break

    for w in range(10, -10, -increase):
        angle = w / 10
        look_pose = Pose([tmp_x, tmp_y, tmp_z], frame="hsrb/" + RobotDescription.current_robot_description.base_link)
        look_pose_in_map = tf_listener.transformPose("/map", look_pose)
        look_point_in_map = look_pose_in_map.pose.position

        LookAtAction(
            [Pose([look_point_in_map.x + angle, look_point_in_map.y + angle, look_point_in_map.z])]).resolve().perform()
        rospy.sleep(1)
        human_p = DetectAction(technique='human', state="start").resolve().perform()
        if human_p is not None:
            tmp_human_pose = human_p
            new_human_p = DetectAction(technique='human', state="stop").resolve().perform()
            print(new_human_p)
            return tmp_human_pose


def demo(step):
    with real_robot:
        rospy.Subscriber("/human_pose", PointStamped, human_pose_cb)
        talk = True
        human_bool = False

        start_pose = robot.get_pose()
        if step <= 0:
            text_to_speech_publisher.pub_now("Starting Restaurant Demo.", talk)
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
            perceive_conf = {'arm_lift_joint': 0.20, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
            pakerino(config=config_for_placing)

            # todo: eign müsste sie schräg stehen weil sonst arm in weg und sie muss auf 0.5 hoch das ist nur zum gucken! nicht fahren
            MoveTorsoAction([0.1]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

        if step <= 1:
            # TODO look around muss gleichzeitig ausgeführt werden, während perception läuft
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
            success = False

            while not success:
                try:
                    # text_to_speech_publisher.pub_now("Please wave you hand. I will come to you", talk)
                    tmp = look_around(1, start_pose)

                    human_point_in_map = tmp.pose.position
                    human_pose_in_map = Pose([human_point_in_map.x, human_point_in_map.y, human_point_in_map.z], frame="map")
                    human_pose_in_bl = tf_listener.transformPose("hsrb/" + RobotDescription.current_robot_description.base_link, human_pose_in_map)


                    #TODO for testing: Usage of query_human

                    #new_human_p = DetectAction(technique='human', state="start").resolve().perform()

                    #new_human_p = DetectAction(technique='human', state="stop").resolve().perform()

                    print("Human Point", tmp)
                    print("Human Point in Map", human_point_in_map)
                    print("Human Pose in Map", human_pose_in_map)
                    print("Human Pose in Bl", human_pose_in_bl)
                    print("Toyas Pose in frame", restaurant.toya_pose)

                    #if human_pose_in_bl is not None:
                     #   human_bool = True
                     #   NavigateAction(target_locations=[Pose([human_pose_in_map.position.x-0.25, human_pose_in_map.position.y-0.25, 0])]).resolve().perform()


                except Exception as e:
                    print(e)
                try:
                    MoveTorsoAction([0]).resolve().perform() # Damit Toya eingefahren ist bevor sie losfährt
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                    plan.perform()
                except SensorMonitoringCondition:
                    text_to_speech_publisher.pub_now("Finding my way. Please wait.", talk)
                    image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 2:
            text_to_speech_publisher.pub_now("Driving.", talk)
            image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
            try:
                plan = Code(lambda: NavigateAction([human_pose_in_map.pose.position.x-0.2, human_pose_in_map.pose.position.y-0.2, 0]).resolve().perform()) >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                print("ABBRUCH ALARM ALARM")
                #TODO schauen, ob move interrupt vielleicht schon in einem Designator festgelegt worden ist
                move.interrupt()



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


demo(0)
