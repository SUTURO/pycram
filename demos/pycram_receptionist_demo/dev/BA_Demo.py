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
            image_switch_publisher.pub_now(ImageEnum.HI.value)

            rospy.sleep(1)
            nlp.get_fav_drink(guest1)
            image_switch_publisher.pub_now(ImageEnum.HI.value)

            TalkingMotion("my favorite drink is oil").perform()

            if drive_to_drinks(guest1.fav_drink):
                drinks = True
            else:
                kitchen = True

        if step <= 2:
            # perceive attributes of guest
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            TalkingMotion("i will show you around now").perform()
            rospy.sleep(2)
            TalkingMotion("please step out of the way and follow me").perform()

        if step <= 3:
            if drinks:
                # guide to drinking area
                NavigateAction([nav_pose_to_drink]).resolve().perform()
                NavigateAction([beverage_pose]).resolve().perform()
                MoveJointsMotion(["head_tilt_joint"], [0.1]).perform()
                LookAtAction([look_person_drinks]).resolve().perform()

                DetectAction(technique='human_receptionist', state="start").resolve().perform()
                HeadFollowMotion(state="start").perform()

                TalkingMotion("here you can get yourself a drink").perform()
                rospy.sleep(1.5)
                TalkingMotion(f"we have {guest1.fav_drink} here").perform()
                rospy.sleep(2)
                TalkingMotion("please come closer again").perform()
                # MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()

            if kitchen:
                # guide to drinking area
                NavigateAction([kitchen_pose]).resolve().perform()
                MoveJointsMotion(["head_tilt_joint"], [0.1]).perform()
                DetectAction(technique='human_receptionist', state="start").resolve().perform()
                HeadFollowMotion(state="start").perform()
                TalkingMotion("this it the kitchen").perform()
                rospy.sleep(1)
                TalkingMotion("here you can get yourself a snack").perform()
                rospy.sleep(2)
                TalkingMotion("please come closer again").perform()
                # MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()

        if step <= 4:

            rospy.sleep(1.5)
            TalkingMotion("i love cleaning up here").perform()
            rospy.sleep(1.5)
            TalkingMotion("what do you do in your free time?").perform()
            rospy.sleep(2)
            nlp.store_and_answer_hobby(guest1)
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            rospy.sleep(1.5)
        if step <= 5:
            # lead to living room
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            DetectAction(technique='human_receptionist', state="stop").resolve().perform()
            TalkingMotion("please step out of the way and follow me").perform()
            if drinks:
                NavigateAction([nav_pose_to_couch]).resolve().perform()
            if kitchen:
                NavigateAction([nav_pose_to_couch_from_kitchen]).resolve().perform()
            NavigateAction([couch_pose_semantik]).resolve().perform()

        if step <= 6:

            # find host in living room
            TalkingMotion("welcome to the living room").perform()
            rospy.sleep(1)
            TalkingMotion("i will find a free place to sit for you").perform()

            LookAtAction([look_couch]).resolve().perform()
            guest_pose = detect_point_to_seat(robot)
            if not guest_pose:
                # TODO: check head movement
                MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                if guest_pose:
                    guest1.set_pose(guest_pose)
                else:
                    TalkingMotion("i am sorry i can not find a seat").perform()
                    rospy.sleep(1)
                    TalkingMotion("please sit down yourself").perform()
                    guest1.set_pose(guest_pose)
            else:
                guest2.set_pose(guest_pose)

        if step <= 7:
            # find free place to sit for guest
            LookAtAction([look_couch]).resolve().perform()
            # TODO: change description to you
            DetectAction(technique='human_receptionist').resolve().perform()
            HeadFollowMotion(state="start").perform()

            if drinks:
                pose_guest = PointStamped()
                pose_guest.header.frame_id = "map"
                pose_guest.point.x = 2.4
                pose_guest.point.y = -2
                pose_guest.point.z = 1.2
                PointingMotion(pose_guest).perform()
                TalkingMotion("behind you is the kitchen").perform()
                rospy.sleep(2)
                TalkingMotion("if you want you can get yourself a snack there").perform()
                rospy.sleep(2)

            if kitchen:
                look_end = Pose([2.4, 2.1, 0.8])
                LookAtAction([look_end]).resolve().perform()
                TalkingMotion("behind me is a table with beverages").perform()
                rospy.sleep(2)
                LookAtAction([look_couch]).resolve().perform()
                TalkingMotion("if you want you can get yourself a drink there").perform()
                rospy.sleep(2)

            TalkingMotion("thank you for your time").perform()


demo(0)
