
# from demos.pycram_receptionist_demo.utils.NLP_new import NLP_Helper
from demos.pycram_receptionist_demo.utils.NLP_Json import NLP_Helper
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

# Declare variables for humans
host = HumanDescription("Jule", fav_drink="topical juice bottle")
host.set_id(1)

guest1 = HumanDescription("Lisa", fav_drink="milk")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

############### important poses #######################
couch_pose_semantik = Pose(position=[3.8, 2.1, 0], orientation=[0, 0, -0.7, 0.7])
look_couch = Pose([3.8, 0.3, 0.75])
look_drinks = Pose([2.3, 4.7, -0.03])
#look_person_drinks = Pose([1.9, 4.1, 1])
look_person_drinks = Pose([1.3, 3.6, 1])

nav_pose_to_drink = Pose([2, 0.6, 0], orientation=[0, 0, 0.7, 0.7])
nav_pose_to_couch = Pose([2.2, 1.95, 0], orientation=[0, 0, -0.7, 0.7])
greet_guest_pose = Pose(position=[1.9, -0.18, 0], orientation=[0, 0, -0.8, 0.5])
beverage_pose = Pose(position=[2.2, 4, 0], orientation=[0, 0, 0.9, 0.3])
########################################################


def demo(step: int, two_guests: Optional[bool] = True):

    with (real_robot):
        rospy.loginfo("start demo at step " + str(step))

        # set neutral pose
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
        MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveJointsMotion(["arm_flex_joint"], [-0.2]).perform()

        if step <= 1:

            # greet first guest
            nlp.welcome_guest(guest1)
            display_info(f"guest name is: {guest1.name}")

        if step <= 2:
            # perceive attributes of guest
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            get_attributes(guest1)
            TalkingMotion("i will show you around now").perform()
            rospy.sleep(2)
            TalkingMotion("please step out of the way and follow me").perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 3:
            NavigateAction([nav_pose_to_drink]).resolve().perform()
            NavigateAction([beverage_pose]).resolve().perform()

            TalkingMotion("here you can get yourself a drink").perform()
            MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()

        if step <= 4:
            LookAtAction([look_person_drinks]).resolve().perform()

            DetectAction(technique='human_receptionist', state="start").resolve().perform()
            HeadFollowMotion(state="start").perform()
            nlp.get_fav_drink(guest1)
            display_info(f"guest favorite drink is: {guest1.fav_drink}")

        if step <= 4:
            # ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion(f"let me see if {guest1.fav_drink} is available").perform()
            LookAtAction([look_drinks]).resolve().perform()

            rospy.sleep(1)
            check_drink_available(guest1)
            rospy.sleep(2)
            TalkingMotion("i love cleaning up this table").perform()

            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            LookAtAction([look_person_drinks]).resolve().perform()
            DetectAction(technique='human_receptionist', state="start").resolve().perform()
            HeadFollowMotion(state="start").perform()
            TalkingMotion("what do you do in your free time?").perform()
            rospy.sleep(1.5)
            nlp.store_and_answer_hobby(guest1)
            if guest1.interests:
                display_info(f"guest interest: {guest1.interests[0]}")

        if step <= 5:
            # lead to living room
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            DetectAction(technique='human', state="stop").resolve().perform()
            TalkingMotion("please step out of the way and follow me").perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()

            NavigateAction([nav_pose_to_couch]).resolve().perform()
            NavigateAction([couch_pose_semantik]).resolve().perform()

        if step <= 6:

            # find host in living room
            TalkingMotion("welcome to the living room").perform()

            # try to find face (of host) in living room
            counter = 0
            while counter < 6:
                detected = detect_host_face(host)
                counter += 1
                if detected:
                    break
                if counter == 1:
                    TalkingMotion("sitting people please look at me").perform()
                    rospy.sleep(1.5)

                elif counter == 2:
                    # look to the side to find face
                    MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                    TalkingMotion("sitting people please look at me").perform()
                    rospy.sleep(1.5)

                if counter == 5:
                    try:
                        start_time = time.time()
                        while start_time < 6 and not host.pose:
                            host_pose = DetectAction(technique='human_receptionist').resolve().perform()
                            host.set_pose(host_pose)

                    except Exception as e:
                        print(e)
                    break

                counter += 1

        if step <= 7:
            # find free place to sit for guest
            LookAtAction([look_couch]).resolve().perform()
            guest_pose = detect_point_to_seat(robot)
            if not guest_pose:
                # look to the side to find seat
                MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                if guest_pose:
                    guest1.set_pose(guest_pose)
                else:
                    TalkingMotion("i am sorry i can not find a seat").perform()
                    guest1.set_pose(guest_pose)
            else:
                guest1.set_pose(guest_pose)

        if not two_guests:
            if host.pose:
                LookAtAction([PoseStamped_to_Pose(host.pose)]).resolve().perform()
                TalkingMotion(f"hey {host.name}, {guest1.name} has arrived").perform()
                rospy.sleep(1.5)
            LookAtAction([PoseStamped_to_Pose(guest1.pose)]).resolve().perform()
            TalkingMotion(f"tell me if you need further assistance").perform()

        if two_guests:
            if step <= 8:
                # introduce sitting people
                TalkingMotion("i will go back to the entrance to assist other guests").perform()
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()


            if step <= 9:
                # go back to start-pose
                NavigateAction([greet_guest_pose]).resolve().perform()
                ParkArmsAction([Arms.LEFT]).resolve().perform()
                MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
                MoveJointsMotion(["arm_flex_joint"], [-0.2]).perform()
                image_switch_publisher.pub_now(ImageEnum.HI.value)

            if step <= 10:
                # greet second guest and lead to living room
                nlp.welcome_guest(guest2)
                display_info(f"guest name is: {guest2.name}")

                MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
                TalkingMotion("i will show you around").perform()
                rospy.sleep(1)
                TalkingMotion("please step out of the way and follow me").perform()
                image_switch_publisher.pub_now(ImageEnum.HI.value)
                MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
                MoveJointsMotion(["head_pan_joint"], [0.0]).perform()

                NavigateAction([nav_pose_to_drink]).resolve().perform()
                NavigateAction([beverage_pose]).resolve().perform()

            if step <= 11:
                TalkingMotion("here you can get a drink").perform()
                MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()

                LookAtAction([look_person_drinks]).resolve().perform()
                DetectAction(technique='human_receptionist', state="start").resolve().perform()
                HeadFollowMotion(state="start").perform()

                nlp.get_fav_drink(guest2)
                display_info(f"guest favorite drink is: {guest2.fav_drink}")
                rospy.sleep(1)

                TalkingMotion(f"let me see if {guest2.fav_drink} is available").perform()
                LookAtAction([look_drinks]).resolve().perform()
                check_drink_available(guest2)

                MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
                LookAtAction([look_person_drinks]).resolve().perform()
                DetectAction(technique='human_receptionist', state="start").resolve().perform()
                HeadFollowMotion(state="start").perform()
                rospy.sleep(1)
                TalkingMotion("i love cleaning up this table").perform()
                rospy.sleep(1.5)
                TalkingMotion("what do you do in your free time?").perform()
                rospy.sleep(1.5)
                nlp.store_and_answer_hobby(guest2)
                display_info(f"guest interest: {guest2.interests[0]}")

            if step <= 12:
                # lead to living room
                MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
                TalkingMotion("i will show you the living room now").perform()
                rospy.sleep(1.5)
                DetectAction(technique='human', state="stop").resolve().perform()
                TalkingMotion("please step out of the way and follow me").perform()
                MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
                MoveJointsMotion(["head_pan_joint"], [0.0]).perform()

                image_switch_publisher.pub_now(ImageEnum.HI.value)
                NavigateAction([nav_pose_to_couch]).resolve().perform()
                NavigateAction([couch_pose_semantik]).resolve().perform()
                TalkingMotion("welcome to the living room").perform()

                # recognise people, did they change seats
                identify_faces(host, guest1)

            if step <= 13:
                # find free place for second guest
                LookAtAction([look_couch]).resolve().perform()
                guest_pose = detect_point_to_seat(robot)
                if not guest_pose:
                    MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                    guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                    if guest_pose:
                        guest2.set_pose(guest_pose)
                    else:
                        TalkingMotion("i am sorry i can not find a seat")
                        # guest1.set_pose(guest_pose)
                        rospy.sleep(2)
                        TalkingMotion("please find a seat yourself")
                        rospy.sleep(1)
                else:
                    guest2.set_pose(guest_pose)

            if step <= 14:
                # introduce everyone and state attributes of first guest
                HeadFollowMotion(state="start").perform()
                rospy.sleep(2)
                introduce(guest1, guest2)
                rospy.sleep(3)
                describe(guest1)
                MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()


demo(0, False)
