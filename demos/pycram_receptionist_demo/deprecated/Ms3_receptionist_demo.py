import rospy
from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from pycram.enums import ImageEnum as ImageEnum
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

world = BulletWorld("DIRECT")
# v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
giskardpy.init_giskard_interface()

# RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# giskardpy.sync_worlds()

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False
doorbell = False

# Declare variables for humans
host = HumanDescription("Celina", fav_drink="coffee")
guest1 = HumanDescription("bob", fav_drink="tea")
# for testing, if the first part of the demo is skipped
guest1.set_attributes(['female', 'without a hat', 'wearing a t-shirt', ' a dark top'])

guest2 = HumanDescription("guest2")
seat_number = 2

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
sound_publisher = SoundRequestPublisher()


def data_cb(data):
    global response
    global callback
    global doorbell

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    response.append("None")
    callback = True


def doorbell_cb(data):
    global doorbell
    doorbell = True


def door_opening():
    TalkingMotion("waiting for guests").resolve().perform()

    # Pre-Pose for door opening
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    pose1 = Pose([1.3, 4.35, 0], [0, 0, 1, 0])
    NavigateAction([pose1]).resolve().perform()
    MoveJointsMotion(["wrist_roll_joint"], [-1.57]).resolve().perform()
    MoveTorsoAction([0.4]).resolve().perform()
    DoorOpenAction("iai_kitchen/living_room:arena:door_handle_inside")

    # move away from door
    pose2 = Pose([1.9, 4.5, 0], [0, 0, 1, 0])
    NavigateAction([pose2]).resolve().perform()


def get_attributes(guest: HumanDescription):
    """
    storing attributes and face of person in front of robot
    :param guest: variable to stare information in
    """
    attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
    guest.set_attributes(attr_list)
    rospy.loginfo(attr_list)

    # remember face
    new_id = DetectAction(technique='human', state='face').resolve().perform()[1][0]
    guest.set_id(new_id)


def welcome_guest(num, guest: HumanDescription):
    global callback

    TalkingMotion("Welcome, please step in front of me").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    # look for human
    DetectAction(technique='human').resolve().perform()

    # look at guest and introduce
    HeadFollowAction('start').resolve().perform()
    TalkingMotion("Hello, i am Toya and my favorite drink is oil").resolve().perform()
    rospy.sleep(3)
    TalkingMotion("please answer me after the sound").resolve().perform()
    rospy.sleep(2)
    TalkingMotion("What is your name and favorite drink?").resolve().perform()
    rospy.sleep(2)

    # signal to start listening
    pub_nlp.publish("start listening")
    rospy.sleep(3)
    image_switch_publisher.pub_now(ImageEnum.TALK.value)
    sound_publisher.publish_sound_request()

    # wait for nlp answer
    while not callback:
        rospy.sleep(1)
    callback = False

    if response[0] == "<GUEST>":
        # success a name and intent was understood
        if response[1] != "<None>":
            TalkingMotion("please confirm if i got your name right").resolve().perform()
            guest.set_drink(response[2])
            rospy.sleep(1)
            guest.set_name(name_confirm(response[1]))

        else:
            # save heard drink
            if response[2]:
                guest.set_drink(response[2])

            # ask for name again once
            guest.set_name(name_repeat())

        # confirm favorite drink
        guest.set_drink(drink_confirm(guest.fav_drink))

    else:
        # two chances to get name and drink
        i = 0
        while i < 2:
            TalkingMotion("please repeat your name and drink loud and clear").resolve().perform()
            pub_nlp.publish("start")
            rospy.sleep(3.5)
            image_switch_publisher.pub_now(ImageEnum.TALK.value)
            sound_publisher.publish_sound_request()

            while not callback:
                rospy.sleep(1)
            callback = False

            if response[0] == "<GUEST>":
                guest.set_name(response[1])
                guest.set_drink(response[2])
                break
            else:
                i += 1

    TalkingMotion("i will show you the living room now").resolve().perform()

    # get attributes and face if first guest
    if num == 1:
        try:
            get_attributes(guest)

        except PerceptionObjectNotFound:
            # failure handling, if human has stepped away
            TalkingMotion("please step in front of me").resolve().perform()
            rospy.sleep(3.5)
            #
            try:
                get_attributes(guest)

            except PerceptionObjectNotFound:
                print("continue without attributes")

    return guest


def demo(step):
    with real_robot:
        global callback
        global guest1

        # signal start
        image_switch_publisher.pub_now(ImageEnum.HI.value)

        # receive data from nlp via topic
        rospy.Subscriber("nlp_out", String, data_cb)
        rospy.Subscriber("nlp_out2", String, doorbell_cb)

        if step <= 0:
            # door opening sequence

            while not doorbell:
                print("no bell")

            door_opening()

        if step <= 1:
            # reception/talking sequence
            guest1 = welcome_guest(1, guest1)

        if step <= 2:
            # leading to livingroom and pointing to free seat

            TalkingMotion("please step out of the way and follow me").resolve().perform()

            # stop looking at human
            HeadFollowAction('stop').resolve().perform()
            DetectAction(technique='human', state='stop').resolve().perform()

            # lead human to living room
            NavigateAction([door_to_couch]).resolve().perform()
            MoveGripperMotion(motion="close", gripper="left").resolve().perform()

            # place new guest in living room
            TalkingMotion("Welcome to the living room").resolve().perform()
        if step <= 3:
            rospy.sleep(4)
            # detect host
            host_pose = DetectAction(technique='human').resolve().perform()
            host.set_pose(host_pose[1])
            host_pose = DetectAction(technique='human', state='stop').resolve().perform()

            # detect free seat
            seat = DetectAction(technique='location', state="sofa").resolve().perform()
            free_seat = False
            for place in seat[1]:
                if place[0] == 'False':
                    PointingMotion(float(place[1]), float(place[2]), float(place[3])).resolve().perform()
                    pose_guest1 = PoseStamped()
                    pose_guest1.header.frame_id = "/map"
                    pose_guest1.pose.position.x = float(place[1])
                    pose_guest1.pose.position.y = float(place[2])
                    pose_guest1.pose.position.z = float(place[3])
                    guest1.set_pose(pose_guest1)
                    break

        if step <= 4:
            # introduce guest1 and host
            HeadFollowAction('start').resolve().perform()
            rospy.sleep(1.2)
            if guest1.pose:
                pub_pose.publish(guest1.pose)
            TalkingMotion("please take a seat next to your host").resolve().perform()
            rospy.sleep(3)

            # introduce humans and look at them
            introduce(host, guest1)
            rospy.sleep(2)

        if step <= 5:
            # describe guest1
            describe(guest1)
            HeadFollowAction('stop').resolve().perform()


demo(0)
