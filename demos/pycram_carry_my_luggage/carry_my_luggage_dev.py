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
            ParkArmsAction(Arms.LEFT).resolve().perform()
            demo_start(human)

        if step <= 2:
            TalkingMotion("Following you.").perform()
            img.pub_now(ImageEnum.FOLLOWSTOP.value)
            TalkingMotion("Push down my Hand, when we arrived.").perform()

            try:
                plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func_human)
                plan.perform()
            except Exception as e:
                print(f"Exception type: {type(e).__name__}")

                if isinstance(e, SensorMonitoringCondition):
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

                        if step <= 3:
                            drive_back_move_base(start_pose)
                elif isinstance(e, HumanNotFoundCondition):
                    lost_human(human)
                else:
                    # TODO idk what giskard throws errors types
                    rospy.logerr("idk what happned")


demo(0)
