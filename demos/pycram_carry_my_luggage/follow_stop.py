import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from demos.pycram_receptionist_demo.utils.helper import *
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import rospy
import subprocess
from pycram.datastructures.enums import ObjectType

from pycram.utilities.robocup_utils import TextToImagePublisher, ImageSwitchPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()
text_to_img_publisher = TextToImagePublisher()
img = ImageSwitchPublisher()
navigation = PoseNavigator()
fts = ForceTorqueSensor(robot_name='hsrb')
rkclient = create_action_client('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
rkclient.wait_for_server()
rospy.loginfo("You can start your demo now")
drive_poses = []


class Human:
    """
    Class that represents humans. This class does not spawn a human in a simulation.
    """

    def __init__(self):
        self.human_pose = False

        # Subscriber to the human pose topic
        self.human_pose_sub = rospy.Subscriber("/human_pose", PointStamped, self.human_pose_cb)

    def human_pose_cb(self, HumanPoseMsg):
        """
        Callback function for human_pose Subscriber.
        Sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic.
        :param HumanPoseMsg: received message
        """
        self.human_pose = True

human = Human()
first_timer_pose = None
second_timer_pose = None
start_time = time.time()
timeout1 = 14


def demo(step: int, clear_path: Optional[bool] = True):
    global start_time
    global first_timer_pose
    global drive_poses

    with (real_robot):
        if step <= 0:
            # move robot in starting position
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            MoveJointsMotion(["arm_roll_joint"], [-1.2]).perform()
            img.pub_now(ImageEnum.HI.value)
            MoveJointsMotion(["head_tilt_joint"], [0.2]).perform()
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()

        if step <= 1:
            # wait for human and hand to be pushed down
            demo_start(human)

        if step <= 2:
            TalkingMotion("i will follow you now until you push down my gripper").perform()
            rospy.sleep(2)
            img.pub_now(ImageEnum.FOLLOWSTOP.value)

            try:
                # perceive and follow human
                plan = Code(lambda: giskardpy.cml(drive_back=False, clear_path=clear_path)) >> Monitor(monitor_func_no_timer)
                plan.perform()
                plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func_no_timer)
                plan.perform()

            except SensorMonitoringCondition:
                MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
                TalkingMotion("stopping").perform()
                demo(1, True)
                
            except giskardpy.ExecutionException:
                TalkingMotion("Wait").perform()
                rospy.sleep(1)
                TalkingMotion("i lost sight of you").perform()
                rospy.sleep(1)
                TalkingMotion("Please come back").perform()
                rospy.sleep(1)
                MoveJointsMotion(["head_tilt_joint"], [0.2]).perform()
                MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
                demo_start(human=human)
                demo(2, clear_path=False)


def demo_start(human: Human):
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """
    global start_time
    try:
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        TalkingMotion("Push down my Hand, when i should follow you").perform()
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func_no_timer)
        plan.perform()

    except SensorMonitoringCondition:
        img.pub_now(ImageEnum.SEARCH.value)
        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()

        TalkingMotion("Please step in front of me").perform()
        human.human_pose = False

        goal_msg = QueryGoal()
        x = rkclient.send_goal(goal_msg)

        # failure handling if no human is detected
        rospy.loginfo("Waiting for human to be detected")
        start_time = time.time()
        timeout = 5
        timeout2 = 15

        while not human.human_pose:
            if time.time() - start_time >= timeout:
                rkclient.send_goal(goal_msg)
            if time.time() - start_time >= timeout2:
                TalkingMotion("please step in front of me").perform()
                start_time = time.time()

        TalkingMotion("Thank you").perform()
        img.pub_now(ImageEnum.HI.value)
        rospy.sleep(2)
        return


def monitor_func_no_timer():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 18.50:
        rospy.logwarn("sensor exception, gripper pushed")
        return SensorMonitoringCondition

    return False


demo(0)
