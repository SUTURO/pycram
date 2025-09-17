from demos.pycram_receptionist_demo.utils.NLP_new import NLP_Helper
from demos.pycram_receptionist_demo.utils.helper import *
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import ImageSwitchPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from demos.pycram_receptionist_demo.utils.ResponseLoader import ResponseLoader
import rospy

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
RobotStateUpdater("/tf", "/giskard_joint_states")
image_switch_publisher = ImageSwitchPublisher()

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")

# variables for communication with nlp
response = [None, None, None]
callback = False
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)
nlp = NLP_Helper()

# response loader
# res_loader = ResponseLoader(json_file='resp.json')
# res_loader.load_data()

# Declare variables for humans
host = HumanDescription("Bob", fav_drink="coffee", interests=["gaming"])
host.set_id(1)

guest1 = HumanDescription("Lisa", fav_drink="water")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

# important poses
couch_pose_semantik = Pose(position=[4.1, 2, 0], orientation=[0, 0, -0.7, 0.7])
look_couch = Pose([4, 0.3, 0.75])
look_drinks = Pose([2.15, 4.7, 0.55])
look_person_drinks = Pose([1.9, 3.8, 1])
nav_pose_to_drink = Pose([2, 0.6, 0], orientation=[0, 0, 0.7, 0.7])
nav_pose_to_couch = Pose([2.2, 3.3, 0], orientation=[0, 0, -0.7, 0.7])
nav_pose_to_couch_from_kitchen = Pose([2.2, -0.8, 0], orientation=[0, 0, 0.7, 0.7])
greet_guest_pose = Pose(position=[1.9, -0.18, 0], orientation=[0, 0, -0.8, 0.5])
beverage_pose = Pose(position=[2.2, 4, 0], orientation=[0, 0, 0.9, 0.3])
kitchen_pose = Pose(position=[3.5, -2.35, 0], orientation=[0, 0, 1, 0])


available_drinks_ba = ["water", "cola", "coffee", "juice", "apple juice", "milk"]


def drive_to_drinks(drink: str):
    drink = drink.strip()
    global drinks
    for i in range(len(available_drinks_ba)):
        if drink == available_drinks_ba[i]:
            return True

    return False


def demo(step: int):
    print(robot.get_pose())
    drinks = False
    kitchen = False

    with (real_robot):
        rospy.loginfo("start demo at step " + str(step))

        # set neutral pose
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveJointsMotion(["arm_flex_joint"], [-0.25]).perform()
        MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()

        if step <= 1:
            # greet first guest
            nlp.welcome_guest(guest1)
            nlp.get_fav_drink(guest1)
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            rospy.sleep(1.5)
            TalkingMotion("i love cleaning up here").perform()
            rospy.sleep(1.5)
            TalkingMotion("what do you do in your free time?").perform()
            rospy.sleep(2)
            nlp.store_and_answer_hobby(guest1)
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            rospy.sleep(1.5)
            describe(guest1)



demo(0)
