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


def demo(step: int):
    with real_robot:

        if step <= 1:
            start_pose = robot.get_pose()
            # ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion("beep boop").perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
            # wait for human and hand to be pushed down
            demo_start(human)
            # DetectAction(technique='human').resolve().perform()
            # rospy.sleep(1.5)

        if step <= 2:
            TalkingMotion("Following you.").perform()
            img.pub_now(ImageEnum.FOLLOWSTOP.value)
            TalkingMotion("Push down my Hand, when we arrived.").perform()

            try:
                plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func_human)
                plan.perform()
            except SensorMonitoringCondition:
                TalkingMotion("We have arrived.").perform()
                img.pub_now(ImageEnum.PUSHBUTTONS.value)
                rospy.sleep(1)
                TalkingMotion("I am not able to pick up the bag. Please hand it in").perform()
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT)
                rospy.sleep(1)

                TalkingMotion("Push down my Hand, when you placed the bag in my gripper").perform()
                img.pub_now(ImageEnum.PUSHBUTTONS.value)
                try:
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                    plan.perform()
                except SensorMonitoringCondition:
                    TalkingMotion("Closing my Gripper.").perform()
                    img.pub_now(ImageEnum.HI.value)
                    MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
                    TalkingMotion("end").perform()
            print("hey")


def monitor_func_human():
    """
    monitors gripper state and if human is still in field of vision
    """
    print("aaaaaaaaa")
    der = fts.get_last_value()
    print(der.wrench.force.x)
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor")
        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        return SensorMonitoringCondition
    if not human.human_pose.get_value():
        rospy.logerr("lost human")
        return HumanNotFoundCondition
    else:
        print("its fine")



demo(0)
