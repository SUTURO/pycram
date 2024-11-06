import rospy

from demos.pycram_clean_the_table_demo.utils.misc import sort_objects
from pycram.designators.action_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool#
from pycram.enums import ImageEnum as ImageEnum
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter


move = PoseNavigator()
pose = Pose([4.2, 2.5, 0])
talk = TextToSpeechPublisher()
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)

world = BulletWorld("DIRECT")
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_5.urdf")
print("before giskard")
giskardpy.init_giskard_interface()

RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# Fallback objects for demo
# muesli_box = Object("Muesli-Box", ObjectType.BREAKFAST_CEREAL)
def data_cb(data):
    global response
    global callback

    # image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    response.append("None")
    callback = True

def demo(step):
    with real_robot:
        global callback
        callback = False
        # Gets the object that HSR should retrieve
        if step <= 0:
            rospy.Subscriber("nlp_out", String, data_cb)
            TalkingMotion("please tell me what i can get you").resolve().perform()
            #talk.pub_now("please say something")
            rospy.sleep(6)
            #pub_nlp.publish("start listening")
            #while not callback:
            #    rospy.sleep(1)

            name = response[1]
            #obj = response[2]
            TalkingMotion(f"I will bring you the Muesli Box, {name}.").resolve().perform()
        # Moves to place
        if step <= 1:

            rospy.sleep(2)
            move.pub_now(pose)
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            # NavigateAction([pose]).resolve().perform()
            rospy.sleep(2)

        # Detects object at goal place
        if step <= 2:
            LookAtAction([Pose([5, 2.55, 0.43])]).resolve().perform()
            shelf_obj = DetectAction(technique='all').resolve().perform()
            obj_list = sort_objects(robot, shelf_obj, ["Mueslibox"])
            print(shelf_obj)

        # Tries to pick up object and changes into goal pose
        if step <= 3:
            PickUpAction(obj_list[0], ["left"], ["front"]).resolve().perform()
            # NavigateAction([Pose([robot.get_pose().pose.position.x - 0.3, robot.get_pose().pose.position.y, 0])]).resolve().perform()
            move.pub_now(Pose([robot.get_pose().pose.position.x - 0.2, robot.get_pose().pose.position.y, 0]))
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            # NavigateAction([Pose([], [])]).resolve().perform()
            move.pub_now(Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y, 0], [0, 0, 1, 0]))
            TalkingMotion(f"Here is your Mueslibox {name}").resolve().perform()




demo(0)