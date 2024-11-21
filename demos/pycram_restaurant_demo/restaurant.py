import math

import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped, PoseWithCovarianceStamped
from robokudo_msgs.msg import QueryAction, QueryGoal

from pycram.datastructures.enums import ObjectType
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import MoveGripperMotion
# from pycram.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.local_transformer import LocalTransformer
# from pycram.plan_failures import SensorMonitoringCondition
# from pycram.pose import Pose
from pycram.process_module import real_robot
# from demos.pycram_hsrb_real_test_demos.utils.startup import startup
import pycram.external_interfaces.giskard as giskardpy
import pycram.external_interfaces.robokudo as robokudo

import rospy
import tf
from geometry_msgs.msg import PoseStamped


from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

# from pycram.ros.viz_marker_publisher import ManualMarkerPublisher

# Initialize the necessary components
# world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()

# giskardpy.init_giskard_interface()
# robokudo.init_robokudo_interface()
# marker = ManualMarkerPublisher()
# listener = tf.TransformListener()
# Initialize the Bullet world for simulation

world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

# robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchen.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)
class Restaurant:
    def __init__(self):
        self.toya_pose = Fluent()
        self.human_pose = None
        self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)

    def toya_pose_cb(self, msg):
        #print("updating")
        self.toya_pose.set_value(robot.get_pose())
        rospy.sleep(0.5)

    def distance(self):
        print("toya pose:" + str(self.toya_pose.get_value().pose))
        if self.human_pose:
            dis = math.sqrt((self.human_pose.pose.position.x - self.toya_pose.get_value().pose.position.x) ** 2 +
                            (self.human_pose.pose.position.y - self.toya_pose.get_value().pose.position.y) ** 2)
            print("dis: " + str(dis))
            return dis
        else:
            rospy.logerr("Cant calculate distance, no human pose found")


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


def monitor_func():
    if restaurant.distance() < 1:
        return SensorMonitoringCondition
    return False


rospy.loginfo("Waiting for action server")
rospy.loginfo("You can start your demo now")


def demo(step):
    with real_robot:
        talk = True

        if step <= 6:
            angle = 0
            while angle <= 1:
                LookAtAction([Pose[robot.get_pose().pose.position.x, robot.get_pose().pose.position.y, 0],
                              [robot.get_pose().pose.orientation.x, robot.get_pose().pose.orientation.y, robot.get_pose().orientation.z, angle]]).resolve().perform()
                angle += 0.5
        start_pose = robot.get_pose()
        if step <= 0:
            text_to_speech_publisher.pub_now("Starting Restaurant Demo.", talk)
            # ParkArmsAction([Arms.LEFT]).resolve().perform()

        if step <= 1:
            # todo look from left to right
            # LookAtAction(targets=[look_pose]).resolve().perform()
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
            success = False
            while not success:
                try:
                    text_to_speech_publisher.pub_now("Please wave you hand. I will come to you", talk)

                    restaurant.human_pose = None
                    human_pose = robokudo.query_waving_human()
                    human_poseTm = transform_pose(human_pose, "head_rgbd_sensor_rgb_frame", "map")
                    human_p = human_poseTm
                    human_p.pose.position.z = 0
                    human_p.pose.orientation.x = 0
                    human_p.pose.orientation.y = 0
                    human_p.pose.orientation.z = 0
                    human_p.pose.orientation.w = 1

                    if human_p:
                        success = True
                        restaurant.human_pose = human_p
                        # print(human_p.pose.position.x)
                except Exception as e:
                    print("An error occurred from perception:", e)
        if step <= 2:
            text_to_speech_publisher.pub_now("Driving", talk)
            image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
            world.current_bullet_world.add_vis_axis(Pose.from_pose_stamped(human_p))
            #print(restaurant.distance())
            marker.publish(Pose.from_pose_stamped(human_p))
            world.current_bullet_world.add_vis_axis(Pose.from_pose_stamped(human_p))
            try:
                plan = Code(lambda: move.pub_now(human_p)) >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                print("ABBRUCH")
                move.interrupt()
            # robot_pose_to_human = Pose.from_pose_stamped(calc_distance(human_p, 0.5))

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


demo(6)
