from datetime import time
from collections import OrderedDict
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import pycram.failures
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from demos.pycram_restaurant_demo.utils.nlp_restaurant import nlp_restaurant
from pycram.datastructures.enums import Arms, ImageEnum
from pycram.datastructures.pose import Pose
# from pycram.demos.pycram_hsrb_real_test_demos.utils.misc_restaurant import Restaurant, monitor_func
from pycram.designators.action_designator import ParkArmsAction, DetectAction, LookAtAction, MoveTorsoAction
from pycram.designators.motion_designator import TalkingMotion, MoveJointsMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.failures import HumanNotFoundCondition
from pycram.language import Code, Monitor
from pycram.process_module import real_robot
from pycram.robot_description import RobotDescription
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition

from pycram.utilities.robocup_utils import pakerino

# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()

rospy.loginfo("Waiting for action server")
rospy.loginfo("You can start your demo now")
response = [None, None]
#confirmation = [None]
callback = False
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)

nlp = nlp_restaurant()
pose_dict = OrderedDict()
###########################################################################

# Initialize global variable
global human_pose
human_pose = None
timeout = 10
customers = list()
global customerCounter
customerCounter = 0
fts = ForceTorqueSensor(robot_name='hsrb')

# Pose required because of multiple customers
#kitchen_pose = Pose([3.66, 2.09, 0.75], [0,0,-0.7, 0.64])
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
    x = -10.0
    while x <= 10.0:
        look_pose = Pose([tmp_x, tmp_y, tmp_z],
                         frame="hsrb/" + RobotDescription.current_robot_description.base_link)
        look_pose_in_map = tf_listener.transformPose("/map", look_pose)
        look_point_in_map = look_pose_in_map.pose.position
        try:
            human_pose = DetectAction(technique='waving', state='start').resolve().perform()
        except pycram.failures.PerceptionObjectNotFound:
            print("oh no, no waving human was found")
        if human_pose:
            break
        else:
            new_pose = Pose([look_point_in_map.x , look_point_in_map.y + x,
                   0.8])
            LookAtAction([new_pose]).resolve().perform()
            rospy.sleep(1.5)

        x += increase
        if x == 10.0:
            x = -10.0

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

        #TalkingMotion("i will start driving now").perform()
        rospy.sleep(2)
        print("done")
        return
def demo(step: int):
    global customer, customerCounter, kitchen_pose, human_pose
    with real_robot:
        talk = True
        start_pose = robot.get_pose()
        kitchen_pose = start_pose
        #confirmation()
        #TalkingMotion("start restaurant demo").perform()

        if step <= 0:
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            #config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
            #                      'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
            #pakerino(config=config_for_placing)
            MoveTorsoAction([0.15]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

        if step <= 1:
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
           # look_around(4, start_pose, talk)
            MoveTorsoAction([0]).resolve().perform()
            MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
            try:
                human_pose = DetectAction(technique='waving', state='start').resolve().perform()
                print(human_pose)
            except pycram.failures.PerceptionObjectNotFound:
                print("No human found")
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            try:
                human_pose = DetectAction(technique='waving', state='start').resolve().perform()
            except pycram.failures.PerceptionObjectNotFound:
                print("No human found")
            MoveJointsMotion(["head_pan_joint"], [0.3]).perform()
            try:
                human_pose = DetectAction(technique='waving', state='start').resolve().perform()
            except pycram.failures.PerceptionObjectNotFound:
                print("No human found")
            if human_pose is not None:
                drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
                image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
                customerCounter += 1
                customer = CustomerDescription(customerCounter, drive_pose)
                customers.append(customer)
            marker.publish(Pose.from_pose_stamped(drive_pose), color=[1, 1, 0, 1], name="human_waving_pose")
            rospy.sleep(2.5)
            move.pub_now(navpose=drive_pose)
            rospy.sleep(1)

        if step <= 2:  # Order step
            image_switch_publisher.pub_now(ImageEnum.ORDER.value)
            MoveTorsoAction([0.1]).resolve().perform()
            LookAtAction([Pose([robot.pose.position.x, robot.pose.position.y, 0.8])])
            rospy.sleep(1)
            Timmi = CustomerDescription(id=1, pose=robot.get_pose())
            #customer = Timmi
            nlp.get_order(customer=customer)
            print(customer.order)
            rospy.sleep(2)
            if customer.order is not None:
                nlp.confirm_order(customer=customer, order=customer.order)
        if step <= 3:  # Drive back step
            rospy.sleep(2)
            TalkingMotion("I will drive back now and return with your order").perform()
            rospy.sleep(2.5)
            image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
            MoveTorsoAction([0]).resolve().perform()
            rospy.sleep(2)
            order_kitchen_pose = Pose([kitchen_pose.pose.position.x, kitchen_pose.pose.position.y, kitchen_pose.pose.position.z],[0,0,-0.7, 3.16])

            move.pub_now(navpose=order_kitchen_pose)

            rospy.sleep(2.5)
            if len(customer.order) == 1:
                TalkingMotion(f"Please prepare the order {customer.order[0][1]} {customer.order[0][0]}").perform()
                rospy.sleep(2)
            elif len(customer.order) > 2:
                TalkingMotion("Please prepare the following order").perform()
                for n in customer.order:
                    TalkingMotion(f"{n[1]}{n[0]}").perform()
            TalkingMotion(f"Please push down my gripper, if the order is prepared").perform()
            confirmation()
            #nlp.give_order(order=customer.order)

            rospy.sleep(3)
            nlp.order_ready()
            TalkingMotion("I will bring the order to the customer now").perform()
        if step <= 4:

            rospy.sleep(2.5)
            move.pub_now(navpose=customer.pose)
            rospy.sleep(2.5)
            nlp.took_order()
            move.pub_now(navpose=kitchen_pose)
            while len(customers) <= 2:
                print(len(customers))
                demo(0)





demo(0)


