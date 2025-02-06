import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_carry_my_luggage.utils.cml_helper import *
from demos.pycram_carry_my_luggage.utils.cml_human import Human
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot
import rospy

# Initialize the necessary components
tf_listener, marker, world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()
human = Human()
start_pose = Pose([1, 1, 0])
# Timer to check for no message




def demo(step: int):
    with real_robot:
        img.pub_now(ImageEnum.HI.value)

        if step <= 1:
            rospy.sleep(1)
            print("start demo")
            start_pose = robot.get_pose()
            print(start_pose)
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            #TalkingMotion("beep boop").perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

            # wait for human and hand to be pushed down
            demo_start(human)


        if step <= 2:
            TalkingMotion("Following you.").perform()
            img.pub_now(ImageEnum.FOLLOWSTOP.value)
            TalkingMotion("Push down my Hand, when we arrived.").perform()

            try:
                plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                TalkingMotion("We have arrived.").perform()
                img.pub_now(ImageEnum.PUSHBUTTONS.value)
                rospy.sleep(1)
                MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
                TalkingMotion("I am not able to pick up the bag. Please hand it in").perform()
                # MoveGripperMotion(GripperState.OPEN, Arms.LEFT)
                rospy.sleep(2)

                TalkingMotion("please place the bag in my gripper and push down my gripper ").perform()
                # TODO: Timer einbauen? falls gripper nicht gedrückt wird

                img.pub_now(ImageEnum.PUSHBUTTONS.value)
                try:
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                    plan.perform()
                except SensorMonitoringCondition:
                    TalkingMotion("Closing my Gripper.").perform()
                    img.pub_now(ImageEnum.HI.value)
                    MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()

                if step <= 3:
                    print(start_pose)
                    drive_back_move_base(start_pose)

            # except HumanNotFoundCondition:
            #     print("whuuuu")
            #     lost_human(human)
            # except e:
            #     print("idk what happened")


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    # print(der.wrench.force.x)
    if abs(der.wrench.force.x) > 10.30:
        #MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        rospy.logwarn("sensor exception")
        #giskardpy.cancel_goal()
        #MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
        return SensorMonitoringCondition

    return False


def lost_human(human: Human):
    """
    function to handle case the robot loses vision of the human it is following
    :param human: variable of human that is lost
    """
    no_human = True
    while no_human:
        goal_msg = QueryGoal()
        if human.human_pose.get_value():
            no_human = False
        else:
            rospy.sleep(5)
            TalkingMotion("I lost you, please step in front of me").perform()
            rkclient.send_goal(goal_msg)


def drive_back_move_base(start_pose):
    """
    navigate with move base to the start point of the challenge
    :param start_pose: pose the robot navigates to
    """
    rospy.loginfo("driving back")
    TalkingMotion("Driving Back.").perform()
    img.pub_now(ImageEnum.DRIVINGBACK.value)
    NavigateAction([start_pose]).resolve().perform()
    TalkingMotion("back at starting position").perform()


def demo_start(human: Human):
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """
    try:

        TalkingMotion("Starting Carry my Luggage demo.").perform()
        img.pub_now(ImageEnum.HI.value)  # hi im toya
        TalkingMotion("Push down my Hand, when you are Ready.").perform()
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        img.pub_now(ImageEnum.HI.value)

        TalkingMotion("Looking for a human").perform()
        DetectAction(technique='human').resolve().perform()
        img.pub_now(ImageEnum.SEARCH.value)
        goal_msg = QueryGoal()
        rkclient.send_goal(goal_msg)
        rospy.loginfo("Waiting for human to be detected")
        no_human = True
        while no_human:

            if human.human_pose.wait_for(timeout=5):
                no_human = False
            else:
                TalkingMotion("Looking for a human, please step in front of me").perform()
                rkclient.send_goal(goal_msg)

        TalkingMotion("Found a Human").perform()
        img.pub_now(ImageEnum.HI.value)
        return






demo(0)
