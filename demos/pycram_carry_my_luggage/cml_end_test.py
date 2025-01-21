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
start_pose = Pose(position=[1.9, -0.18, 0], orientation=[0, 0, -0.8, 0.5])


def demo(step: int):
    with real_robot:
        if step <= 1:
            TalkingMotion("We have arrived.").perform()

            img.pub_now(ImageEnum.PUSHBUTTONS.value)
            rospy.sleep(1)
            TalkingMotion("I am not able to pick up the bag. Please hand it in").perform()
        if step <= 2:
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

demo(0)