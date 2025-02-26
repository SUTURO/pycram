import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot
import rospy

from pycram.utilities.robocup_utils import TextToImagePublisher, ImageSwitchPublisher

# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()
text_to_img_publisher = TextToImagePublisher()
img = ImageSwitchPublisher()
fts = ForceTorqueSensor(robot_name='hsrb')
rkclient = create_action_client('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
rkclient.wait_for_server()
rospy.loginfo("You can start your demo now")

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
start_pose = Pose([1, 1, 0])
first_timer_pose = None
second_timer_pose = None
start_time = time.time()
timeout1 = 15
timeout2 = 25


def demo(step: int):
    global start_time
    global start_pose
    global first_timer_pose
    global  second_timer_pose
    with (real_robot):
        TalkingMotion("Starting Carry my Luggage demo.").perform()
        # MoveJointsMotion(["wrist_roll_joint"], [0.0]).perform()
        # MoveJointsMotion(["arm_roll_joint"], [-1.2]).perform()
        img.pub_now(ImageEnum.HI.value)

        if step <= 1:
            rospy.sleep(1)
            print("start demo")

            # store pose to drive back to
            start_pose = robot.get_pose()
            print("###########################")
            print(start_pose)
            print("###########################")
            text_to_img_publisher.pub_now("Push Down my Gripper")
            img.pub_now(ImageEnum.GENERATED_TEXT.value)


            # move robot in starting position
            # ParkArmsAction([Arms.LEFT]).resolve().perform()
            # MoveJointsMotion(["head_tilt_joint"], [0.2]).perform()
            # MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            # MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
            # #
            # MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

            # wait for human and hand to be pushed down
            demo_start(human)

        if step <= 2:
            TalkingMotion("Following you.").perform()
            img.pub_now(ImageEnum.FOLLOWSTOP.value)
            TalkingMotion("Push down my Hand, when we arrived.").perform()

            try:
                # start timer /store time when following part starts
                start_time = time.time()

                # perceive and follow human
                plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func)
                plan.perform()

            except SensorMonitoringCondition:
                TalkingMotion("We have arrived.").perform()
                text_to_img_publisher.pub_now("please hand the bag in my gripper")
                rospy.sleep(1)
                MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
                img.pub_now(ImageEnum.GENERATED_TEXT.value)
                TalkingMotion("I am not able to pick up the bag. Please hand it in my gripper").perform()
                text_to_img_publisher.pub_now("when the bag is handed in push down my gripper")
                rospy.sleep(3)
                img.pub_now(ImageEnum.GENERATED_TEXT.value)
                TalkingMotion("please put the bag in my gripper and push down my gripper").perform()
                # TODO: Timer einbauen? falls gripper nicht gedrückt wird
                try:
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func_no_timer)
                    plan.perform()
                except SensorMonitoringCondition:
                    TalkingMotion("Closing my Gripper.").perform()
                    img.pub_now(ImageEnum.HI.value)
                    MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
                    if step <= 3:
                        # drive back starting with last recorded pose
                        drive_back_move_base(start_pose, first_timer_pose, second_timer_pose)
            except giskardpy.ExecutionException:
                TalkingMotion("Wait").perform()
                rospy.sleep(1)
                TalkingMotion("i lost sight of you").perform()
                rospy.sleep(1)
                TalkingMotion("Please come back").perform()
                rospy.sleep(1)
                TalkingMotion("i will start following you again when you push my gripper").perform()
                display_info("step in front of me and push my gripper")
                MoveJointsMotion(["head_tilt_joint"], [0.2]).perform()
                MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
                demo_start(human)
                demo(2)


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    global start_time
    global first_timer_pose
    global second_timer_pose
    global timeout1
    global timeout2
    global stored_poses
    der = fts.get_last_value()
    # print(der.wrench.force.x)
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor exception")
        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        return SensorMonitoringCondition
    if not first_timer_pose:
        if int(time.time() - start_time) >= timeout1:
            first_timer_pose = robot.get_pose()
            rospy.loginfo("stored first drive pose")
            print("###########################")
            print(first_timer_pose)
            print("###########################")

    if not second_timer_pose:
        if int(time.time() - start_time) >= timeout2:
            second_timer_pose = robot.get_pose()
            stored_poses = True
            rospy.loginfo("stored second drive pose")
            print("###########################")
            print(second_timer_pose)
            print("###########################")

    return False

def display_info(info: str):
    text_to_img_publisher.pub_now(info)
    rospy.sleep(1)
    img.pub_now(ImageEnum.GENERATED_TEXT.value)
    img.pub_now(ImageEnum.GENERATED_TEXT.value)


def drive_back_move_base(start_pose, first_pose, second_pose):
    """
    navigate with move base to the start point of the challenge
    :param start_pose: pose the robot navigates to
    """
    rospy.loginfo("driving back")
    TalkingMotion("Driving Back.").perform()
    img.pub_now(ImageEnum.DRIVINGBACK.value)

    # was track long enough for second pose to be stored
    if second_pose:
        get_orientation(second_pose, first_pose)
        # drive to the closest known pose
        NavigateAction([second_pose]).resolve().perform()
        print("second pose:")
        print(second_pose)
    if first_pose:
        print("first pose:")
        print(first_pose)
        get_orientation(first_pose, start_pose)
        NavigateAction([first_pose]).resolve().perform()

    NavigateAction([start_pose]).resolve().perform()
    TalkingMotion("back at starting position").perform()


def get_orientation(goal: Pose, pose_after_goal: Pose):
    x_val = pose_after_goal.pose.position.x - goal.pose.position.x
    y_val = pose_after_goal.pose.position.y - goal.pose.position.y

    if abs(x_val) > abs(y_val):
        print("x_val bigger: " + str(x_val))
        if x_val > 0:
            goal.orientation.x = 0
            goal.orientation.y = 0
            goal.orientation.w = 0
            goal.orientation.z = 1
        else:
            goal.orientation.x = 0
            goal.orientation.y = 0
            goal.orientation.w = 1
            goal.orientation.z = 0
    else:
        print("y_val bigger: " + str(x_val))
        if y_val > 0:
            goal.orientation.x = 0
            goal.orientation.y = 0
            goal.orientation.w = 0.7
            goal.orientation.z = 0.7
        else:
            goal.orientation.x = 0
            goal.orientation.y = 0
            goal.orientation.w = -0.7
            goal.orientation.z = 0.7


def demo_start(human: Human):
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """
    global start_time
    try:
        img.pub_now(ImageEnum.HI.value)
        TalkingMotion("Push down my Hand, when you are Ready.").perform()
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func_no_timer)
        plan.perform()

    except SensorMonitoringCondition:
        img.pub_now(ImageEnum.HI.value)

        TalkingMotion("Looking for a human").perform()
        human.human_pose = False
        img.pub_now(ImageEnum.SEARCH.value)
        goal_msg = QueryGoal()
        x = rkclient.send_goal(goal_msg)
        print(x)
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


        TalkingMotion("Found a Human").perform()
        img.pub_now(ImageEnum.HI.value)
        return


def monitor_func_no_timer():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor exception, gripper pushed")
        #MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        return SensorMonitoringCondition

    return False





demo(0)
