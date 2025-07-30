import math
import threading
import time
from collections import OrderedDict
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_matrix

import pycram
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from demos.pycram_restaurant_demo.utils import misc

from pycram.demos.pycram_give_me_a_hand_demo.misc.nlp_gmah import NLP_GMAH
from pycram.designators.motion_designator import *
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from pycram.datastructures.enums import Arms, ImageEnum
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, DetectAction, LookAtAction, MoveTorsoAction, \
    NavigateAction, PlaceGivenObjectAction
from pycram.designators.motion_designator import TalkingMotion, MoveJointsMotion
from pycram.designators.object_designator import CustomerDescription
from pycram.external_interfaces.robokudo import get_used_annotator_list
from pycram.failures import HumanNotFoundCondition
from pycram.language import Code, Monitor
from pycram.process_module import real_robot
from pycram.robot_description import RobotDescription
from pycram.failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from demos.pycram_restaurant_demo.utils.NLP.nlp_main import RestaurantNLP
from pycram.utilities.robocup_utils import pakerino, TextToImagePublisher, ImageSendPublisher

# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()
text_to_img_publisher = TextToImagePublisher()
rospy.loginfo("Waiting for action server")
rospy.loginfo("You can start your demo now")
response = [None, None]
stopped = False
#
callback = False
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)

###########################################################################

# Initialize global variable
global instructor_pose
global instructor_found
global pointing_pose
global pointing_found
# Initialize NLP variables
callback = False
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)
nlp = NLP_GMAH()
response = [None]
nlpInstructor = False

instructor_pose  = None
pointing_pose = None
instructor_found = False
pointing_found = False
timeout = 10
fts = ForceTorqueSensor(robot_name='hsrb')
placingTest = Pose([5.36, 1.57, 1], [0, 0, 0, 1])
placingPoseTest = Pose([5.36, 1.57, 0.75], [0,0,0,1])
objectGoals = []
class FixedRoomPositions(Enum):
    LIVING_ROOM = Pose([1.86, 2.59, 0], [0,0, -1, 1])
    KITCHEN = Pose([2.16, -1.93, 0], [0,0,0,1])
    WORKING_AREA = Pose([3.19, 2.46, 0], [0, 0, 0, 1])

def move_pose_forwards(goal: Pose, distance: float):
    """
    Moves the goal pose of a navigation action on a vector given the set distance.
    :param goal: Pose to move.
    :param distance: Distance to move the goal.
    :return: Moved Pose.
    """
    rotMatrix = quaternion_matrix([goal.pose.orientation.x,
                                   goal.pose.orientation.y,
                                   goal.pose.orientation.z,
                                   goal.pose.orientation.w])
    forward_vector = rotMatrix[:3, 0]
    movedPose = np.array([goal.pose.position.x,
                      goal.pose.position.y,
                      goal.pose.position.z]) - distance * forward_vector
    return movedPose

def set_pose_in_front(goalPose: Pose, dist : float):
    """
    Creates a pose in front of the received goal. Changes the orientation to the goal pose orientation.
    :param: goalPose: transformed goal pose of customer
    :param: dist: The cm in which the pose has to be moved in fron
    :return: newly moved pose
    """
    new_pos = move_pose_forwards(goalPose, dist)
    adjusted_pose = Pose(position=[new_pos[0], new_pos[1], new_pos[2]], orientation=[goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])
    return adjusted_pose


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


def look_around(increase: float, star_pose: PoseStamped, talk=True):
    """
    Make robot look continuously from left to right. Stops if a human is perceived.    :param: increase: The increments in which Toya should look around.
    """

    global instructor_pose ,instructor_found
    instructor_pose = None
    tmp_x = star_pose.pose.position.x
    tmp_y = star_pose.pose.position.y
    tmp_z = star_pose.pose.position.z
    x = -0.5
    tries = 0
    MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
    MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()

    while x <= 1 and tries <= 2:
        print("Tries: " , tries)
        MoveJointsMotion(["head_pan_joint"], [x]).perform()
        try:
            print("hallo z1")
            instructor_pose = DetectAction(technique='gmahWaving', state='start').resolve().perform()
            print("halloe2")
            print(instructor_pose)
        except pycram.failures.PerceptionObjectNotFound:
            print("oh no, no waving human was found")
        if instructor_pose:
            TalkingMotion("I found an instructor").perform()
            instructor_found = True
            break

        x += increase
        if x == 1:
            tries += 1
            x = -0.5
def callOutInstructor() -> bool:
    test = nlp.check_Instructor()
    return test

def searching_for_instructor() -> Pose:
    #TODO transform pose from hgdb camera to map frame

    global instructor_pose
    TalkingMotion("PLease raise your hand to identify yourself as my instructor").perform()
    rospy.sleep(2)
    while not instructor_found:
        NavigateAction([FixedRoomPositions.LIVING_ROOM.value]).resolve().perform()
        callOutInstructor()
        look_around(0.5, robot.get_pose())
        if instructor_found:
            return instructor_pose
        NavigateAction([FixedRoomPositions.KITCHEN.value]).resolve().perform()
        callOutInstructor()
        #MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
        look_around(0.5, robot.get_pose())
        if instructor_found:
            return instructor_pose
        NavigateAction([FixedRoomPositions.WORKING_AREA.value]).resolve().perform()
        callOutInstructor()
        #MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
        look_around(0.5, robot.get_pose())
        if instructor_found:
            return instructor_pose
    if instructor_found:
        return instructor_pose

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

def placeObject(goal_Pose: Pose):
    x_pos = goal_Pose.pose.position.x
    y_pos = goal_Pose.pose.position.y
    z_pos = goal_Pose.pose.position.z
    MoveTorsoAction([0.2]).resolve().perform()

    TalkingMotion("PLacing Object now").perform()
    try:
        PlaceGivenObjectAction(["Crackerbox"], [Arms.LEFT], [Pose([x_pos, y_pos, z_pos])], [Grasp.FRONT], [True]).resolve().perform()
        placed = True
    except pycram.failures.ManipulationFTSCheckNoObject:
        TalkingMotion("Oh no, It seems I can not reach my Placing Pose").perform()
        rospy.sleep(2)
        TalkingMotion("I will open my gripper and let go of the object").perform()
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        TalkingMotion("I will go back now to my instructor ").perform()


def search_location() -> Pose:
    global pointing_pose, pointing_found
    MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
    MoveTorsoAction([0.2]).perform()

    try:

        pointing_pose = DetectAction(technique='pointing', state='start').resolve().perform()
    except pycram.failures.PerceptionObjectNotFound:
        rospy.logwarn("The location is not known to me")
        TalkingMotion("Sorry, I am not sure where I should put it").perform()
        rospy.sleep(2)
        TalkingMotion("so I will bring it to my favourite table ").perform()
        rospy.sleep(2)
        pointing_pose = placingTest
    if pointing_pose:
        pointing_found = True
        TalkingMotion("I will try to place the object now").perform()
        rospy.sleep(1)

def demo(step: int):
     global instructor_pose, instructor_found, pointing_pose, pointing_found
     with real_robot:
        MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
        MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
        config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                               'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
        pakerino(config=config_for_placing)
        MoveTorsoAction([0.2]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Give me a Hand is starting.").perform()
        #MoveJointsMotion(["head_pan_joint"], [0.0]).perform()

        #Search for Instructor inside of map
        if step <= 0:
            tries = 0
            while tries <= 4 and instructor_pose is None:

                searching_for_instructor()
                tries += 1
            if instructor_pose:
                mapInstructorPose = transform_camera_to_x(instructor_pose,"head_rgbd_sensor_link" )
                print("Hello", mapInstructorPose)#o9
                newInstructor = set_pose_in_front(mapInstructorPose, 0.4)
                print(newInstructor)
                NavigateAction([mapInstructorPose]).resolve().perform()
                marker.publish(Pose.from_pose_stamped(mapInstructorPose), color=[1, 1, 0, 1], name="human_waving_pose")
                marker.publish(Pose.from_pose_stamped(newInstructor), color=[1, 0, 1, 1], name="adjusted_pose")
        if step <= 1:
            # Getting Object step
            TalkingMotion("Please place the object into my gripper and push down when my display changes").perform()
            rospy.sleep(1)
            image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)
            try:
                plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                image_switch_publisher.pub_now(ImageEnum.HI.value)
                TalkingMotion("I will close my Gripper now. Please be careful").perform()
                rospy.sleep(1.5)
                MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()

            rospy.sleep(1)
        if step <= 2:
            # Finding goal location
            TalkingMotion("Please point to the location where I should put the object").perform()
            rospy.sleep(2)
            # Testing purposes
            pointing_pose = True
            while not pointing_found:
                search_location()

            if pointing_found:
                if pointing_pose is not None:
                    objectGoals.append(pointing_pose)
                print("Hallo")
                #newPose = transform_camera_to_x(placingTest,"head_rgbd_sensor_link" )
                newPose = set_pose_in_front(pointing_pose, 0.5)
                print(newPose)
                NavigateAction([newPose]).resolve().perform()
                marker.publish(Pose.from_pose_stamped(newPose), color=[1, 0, 1, 1], name="adjusted_pose")
                placeObject(placingPoseTest)
        if step <= 3:
            NavigateAction([mapInstructorPose]).resolve().perform()
            TalkingMotion("I am ready to assist again").perform()
            while len(objectGoals) <= 3:
                demo(1)










demo(2)





