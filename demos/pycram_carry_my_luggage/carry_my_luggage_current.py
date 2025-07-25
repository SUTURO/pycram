import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from demos.pycram_receptionist_demo.utils.helper import *
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import rospy
from pycram.utilities.robocup_utils import TextToImagePublisher, ImageSwitchPublisher


# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot, kitchen = startup()
text_to_img_publisher = TextToImagePublisher()
img = ImageSwitchPublisher()
navigation = PoseNavigator()
fts = ForceTorqueSensor(robot_name='hsrb')
rkclient = create_action_client('robokudo/query', QueryAction)
rkclient.wait_for_server()
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
start_time = time.time()
timeout1 = 14


def demo(step: int, clear_path: Optional[bool] = True):
    global start_time
    global drive_poses

    with (real_robot):
        if step <= 1:
            # TalkingMotion("Starting Carry my Luggage demo.").perform()
            MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            MoveJointsMotion(["arm_roll_joint"], [-1.2]).perform()
            img.pub_now(ImageEnum.HI.value)
            print("start demo")

            # store pose to drive back to with rotated orientation
            start_pose = robot.get_pose()
            print(f"start pose: " + str(start_pose))

            # add start pose but 180 degree rotated
            rotated_quaternion = multiply_quaternions(start_pose.orientation_as_list(), [0, 0, 1, 0])
            start_pose.set_orientation(rotated_quaternion)
            drive_poses.append(start_pose)

            # wait for human and hand to be pushed down
            demo_start(human)

        if step <= 2:
            TalkingMotion("when we arrive, push down my gripper.").perform()
            rospy.sleep(2.5)
            TalkingMotion("please walk slowly i will follow you").perform()
            img.pub_now(ImageEnum.FOLLOWSTOP.value)

            try:
                # start timer /store time when following part starts
                start_time = time.time()

                # perceive and follow human
                plan = Code(lambda: giskardpy.cml(drive_back=False, clear_path=clear_path)) >> Monitor(monitor_func)
                plan.perform()
                plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func_no_timer)
                plan.perform()

            except SensorMonitoringCondition:
                MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
                TalkingMotion("We have arrived.").perform()
                MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()
                text_to_img_publisher.pub_now("please hand the bag in my gripper")
                rospy.sleep(1)
                img.pub_now(ImageEnum.GENERATED_TEXT.value)
                TalkingMotion("I am not able to pick up the bag. Please hand it in my gripper").perform()
                text_to_img_publisher.pub_now("when the bag is handed in push down my gripper")
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
                rospy.sleep(4)
                img.pub_now(ImageEnum.GENERATED_TEXT.value)
                TalkingMotion("please put the bag in my gripper and push down my gripper").perform()
                # TODO: Timer einbauen? falls gripper nicht gedrückt wird
                try:
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func_no_timer)
                    plan.perform()
                except SensorMonitoringCondition:
                    MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
                    TalkingMotion("Closing my Gripper.").perform()
                    MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
                    if step <= 3:
                        # drive back starting with last recorded pose
                        drive_back_move_base()
                        TalkingMotion("back at starting position").perform()
                        img.pub_now(ImageEnum.HI.value)
                        MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
                        rospy.sleep(2)
                        TalkingMotion("completed the task").perform()

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
        rospy.sleep(1)
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


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    this monitor function additionally stores robots position
    """
    global start_time
    global timeout1
    global drive_poses
    der = fts.get_last_value()

    if abs(der.wrench.force.x) > 18.30:
        rospy.logwarn("sensor exception")
        return SensorMonitoringCondition

    # every 14 second new pose store
    if int(time.time() - start_time) >= timeout1:
        # store pose for way back with orientation turned 180 degree
        drive_pose = robot.get_pose().copy()
        rotated_quaternion = multiply_quaternions(drive_pose.orientation_as_list(), [0, 0, 1, 0])
        drive_pose.set_orientation(rotated_quaternion)
        drive_poses.append(drive_pose)
        rospy.loginfo("stored drive pose")
        start_time = time.time()

    return False


def drive_back_move_base():
    """
    navigate with move base to the start point of the challenge
    """

    giskardpy.turning_around()
    rospy.loginfo("driving back")
    TalkingMotion("Driving Back.").perform()
    img.pub_now(ImageEnum.DRIVINGBACK.value)
    rospy.sleep(2)

    # drive back to all collected poses
    drive_poses.reverse()
    for pose in drive_poses:
        NavigateAction([pose]).resolve().perform()


demo(0)
