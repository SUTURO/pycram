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
import rospy
print("start")
# Initialize the Bullet world for simulation
world = BulletWorld()
print("a")
# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()
print("b")
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
print("c")
RobotStateUpdater("/tf", "/giskard_joint_states")
print("d")
image_switch_publisher = ImageSwitchPublisher()
print("e")
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")
print("f")
# variables for communication with nlp
response = [None, None, None]
callback = False
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)
nlp = NLP_Helper()

# Declare variables for humans
host = HumanDescription("James", fav_drink="iced tea")
host.set_id(1)

guest1 = HumanDescription("Lisa", fav_drink="milk")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])
fts = ForceTorqueSensor(robot_name='hsrb')

############### important poses #######################
couch_pose_semantik = Pose(position=[6.6, 5, 0], orientation=[0, 0, 0.34, 0.93])
look_couch = Pose([8.8, 6.6, 0.65])
look_drinks = Pose([8.9, 2.2, 0.25])
look_person_drinks = Pose([7.6, 2.3, 1])
nav_pose_to_drink1 = Pose([3, 0.2, 0], orientation=[0, 0, 0, 1])
nav_pose_to_drink = Pose([7.7, 0.2, 0], orientation=[0, 0, 0.7, 0.7])
nav_pose_to_couch = Pose([6.8, 3.4, 0], orientation=[0, 0, 0.7, 0.7])
greet_guest_pose = Pose(position=[2.5, 0.1, 0], orientation=[0, 0, 1, 0])
couch_to_start = Pose(position=[3.7, 2.6, 0], orientation=[0, 0, -0.7, 0.7])
beverage_pose = Pose(position=[8.2, 3.2, 0], orientation=[0, 0, -0.7, 0.7])
########################################################

def demo(step: int):
    with (real_robot):
        rospy.loginfo("start demo at step " + str(step))
        # NavigateAction([greet_guest_pose]).resolve().perform()

        # set neutral pose
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        #MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
        #MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
        #ParkArmsAction([Arms.LEFT]).resolve().perform()
        # TalkingMotion("starting").perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        MoveJointsMotion(["head_tilt_joint"], [0.1]).perform()

        if step <= 1:
            try:
                img.pub_now(ImageEnum.PUSHBUTTONS.value)
                TalkingMotion("Push down my Hand, when the challenge should start").perform()
                plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
                plan.perform()

            except SensorMonitoringCondition:
                print("start demo with gripper push")

            # greet first guest
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            nlp.welcome_guest(guest1)
            display_info(f"guest name is: {guest1.name}")
            rospy.sleep(1)
            image_switch_publisher.pub_now(ImageEnum.CLOCK.value)
            TalkingMotion("what do you do in your free time?").perform()
            rospy.sleep(1.5)
            nlp.store_and_answer_hobby(guest1)
            rospy.sleep(1)
            if guest1.interests:
                display_info(f"guest interest: {guest1.interests[0]}")
                if guest1.interests[0] == "robots":
                    TalkingMotion("the host likes robots as well").perform()
            else:
                guest1.add_interests("robots")
                display_info(f"guest interest: {guest1.interests[0]}")
                TalkingMotion("the host likes robots as well").perform()


        if step <= 2:
            # perceive attributes of guest
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            get_attributes(guest1)
            TalkingMotion("i will show you around now").perform()
            rospy.sleep(2)
            # ParkArmsAction([Arms.LEFT]).resolve().perform()
            MoveJointsMotion(["arm_roll_joint"], [-1.2]).perform()
            TalkingMotion("please step out of the way and follow me").perform()
            # MoveJointsMotion(["arm_flex_joint"], [-0.25]).perform()
            image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 3:
            NavigateAction([nav_pose_to_drink]).resolve().perform()
            NavigateAction([beverage_pose]).resolve().perform()

            TalkingMotion("here you can get yourself a drink").perform()
            MoveJointsMotion(["arm_flex_joint"], [-0.18]).perform()
            MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()

            # TODO: look at person pose
            LookAtAction([look_person_drinks]).resolve().perform()

            DetectAction(technique='human', state="start").resolve().perform()
            HeadFollowMotion(state="start").perform()
            nlp.get_fav_drink(guest1)
            # display_info(f"guest favorite drink is: {guest1.fav_drink}")
            rospy.sleep(2)

        if step <= 4:
            print("x")
            # ParkArmsAction([Arms.LEFT]).resolve().perform()
            # TalkingMotion(f"let me see if {guest1.fav_drink} is available").perform()
            # MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()

            # TODO: look at drinks?
            #MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            #LookAtAction([look_drinks]).resolve().perform()

            #rospy.sleep(1)
            #check_drink_available(guest1)
            #rospy.sleep(2)

            # TODO: look at person pose
            #LookAtAction([look_person_drinks]).resolve().perform()
            #DetectAction(technique='human', state="start").resolve().perform()
            #HeadFollowMotion(state="start").perform()

        if step <= 5:
            # lead to living room
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            MoveJointsMotion(["arm_flex_joint"], [0.0]).perform()
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            image_switch_publisher.pub_now(ImageEnum.HI.value)
            DetectAction(technique='human', state="stop").resolve().perform()
            TalkingMotion("please step out of the way and follow me").perform()
            # MoveJointsMotion(["head_pan_joint"], [0]).perform()

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
                    MoveJointsMotion(["head_pan_joint"], [-0.7]).perform()
                    TalkingMotion("sitting people please look at me").perform()
                    rospy.sleep(1.5)

                elif counter == 3:
                    # look to the side to find face
                    MoveJointsMotion(["head_pan_joint"], [0.9]).perform()
                    rospy.sleep(1.5)

                if counter == 5:
                    try:
                        rospy.logerr("host has no id")
                        host_pose = DetectAction(technique='human').resolve().perform()
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
                MoveJointsMotion(["head_pan_joint"], [-0.7]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                if guest_pose:
                    guest1.set_pose(guest_pose)
                else:
                    MoveJointsMotion(["head_pan_joint"], [0.7]).perform()
                    guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                    if guest_pose:
                        guest1.set_pose(guest_pose)
                    else:
                        TalkingMotion("i am sorry i can not find a seat")
                        guest1.set_pose(guest_pose)
            else:
                guest1.set_pose(guest_pose)

        if step <= 8:
            # introduce sitting people
            TalkingMotion("i will go back to the entrance to assist other guests").perform()
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            NavigateAction([couch_to_start]).resolve().perform()
            TalkingMotion("i am speed").perform()

        if step <= 9:
            # go back to start-pose
            # TODO: navigate start pose
            NavigateAction([greet_guest_pose]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            MoveJointsMotion(["head_tilt_joint"], [0.1]).perform()
            image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 10:
            # greet second guest and lead to living room
            nlp.welcome_guest(guest2)
            display_info(f"guest name is: {guest1.name}")
            rospy.sleep(1)
            image_switch_publisher.pub_now(ImageEnum.CLOCK.value)
            TalkingMotion("what do you do in your free time?").perform()
            rospy.sleep(1.5)
            nlp.store_and_answer_hobby(guest2)
            rospy.sleep(1)
            if guest2.interests:
                display_info(f"guest interest: {guest2.interests[0]}")
            else:
                guest1.add_interests("robots")
                display_info(f"guest interest: {guest2.interests[0]}")

            describe(guest1)
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            MoveJointsMotion(["arm_flex_joint"], [0.0]).perform()
            TalkingMotion("i will show you around").perform()
            rospy.sleep(1)
            TalkingMotion("please step out of the way and follow me").perform()
            image_switch_publisher.pub_now(ImageEnum.HI.value)

            # TODO: navigate to drinks
            NavigateAction([nav_pose_to_drink]).resolve().perform()
            TalkingMotion("almost there").perform()
            NavigateAction([beverage_pose]).resolve().perform()

        if step <= 11:
            TalkingMotion("here you can get a drink").perform()
            MoveJointsMotion(["arm_flex_joint"], [-0.18]).perform()
            MoveJointsMotion(["torso_lift_joint"], [0.07]).perform()

            #LookAtAction([look_person_drinks]).resolve().perform()
            MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
            DetectAction(technique='human', state="start").resolve().perform()
            HeadFollowMotion(state="start").perform()

            nlp.get_fav_drink(guest2)
            # display_info(f"guest favorite drink is: {guest2.fav_drink}")
            rospy.sleep(2)

            # TalkingMotion(f"let me see if {guest2.fav_drink} is available").perform()
            # MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
            # MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()

            # TODO: look at drinks
            # LookAtAction([look_drinks]).resolve().perform()
            # check_drink_available(guest2)

            # TODO: look at person
            # LookAtAction([look_person_drinks]).resolve().perform()
            # DetectAction(technique='human', state="start").resolve().perform()
            # HeadFollowMotion(state="start").perform()
            # rospy.sleep(1)



        if step <= 12:
            # lead to living room
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            DetectAction(technique='human', state="stop").resolve().perform()
            TalkingMotion("please step out of the way and follow me").perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

            image_switch_publisher.pub_now(ImageEnum.HI.value)

            NavigateAction([couch_pose_semantik]).resolve().perform()
            TalkingMotion("welcome to the living room").perform()

            # recognise people, did they change seats
            identify_faces(host, guest1)

        if step <= 13:
            # find free place for second guest
            LookAtAction([look_couch]).resolve().perform()
            guest_pose = detect_point_to_seat(robot)
            if not guest_pose:
                # TODO: check head movement
                MoveJointsMotion(["head_pan_joint"], [-0.7]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                if guest_pose:
                    guest1.set_pose(guest_pose)
                else:
                    MoveJointsMotion(["head_pan_joint"], [0.9]).perform()
                    guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                    if guest_pose:
                        guest1.set_pose(guest_pose)
                    else:
                        TalkingMotion("i am sorry i can not find a seat")
                        guest1.set_pose(guest_pose)
            else:
                guest2.set_pose(guest_pose)

        if step <= 14:
            # introduce everyone and state attributes of first guest
            HeadFollowMotion(state="start").perform()
            rospy.sleep(2)
            introduce(guest1, guest2)
            rospy.sleep(3)
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            rospy.sleep(2)
            TalkingMotion("have fun at the party").perform()


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 18.50:
        rospy.logwarn("sensor exception, gripper pushed")
        return SensorMonitoringCondition

    return False

demo(0)
