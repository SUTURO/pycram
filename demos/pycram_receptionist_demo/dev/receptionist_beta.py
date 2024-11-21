import rospy

from demos.pycram_receptionist_demo.utils.misc import *
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import ImageSwitchPublisher
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

# robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

callback = False

# Declare variables for humans
host = HumanDescription("Bob", fav_drink="Milk")
host.set_id(1)
guest1 = HumanDescription("Lisa", fav_drink="water")

# for testing, if the first part of the demo is skipped
guest1.set_attributes([[1], ['female', 'without a hat', 'wearing a t-shirt', ' a dark top']])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
# for testing, if the first part of the demo is skipped
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

# important poses
couch_pose_semantik = Pose(position=[3.8, 1.9, 0], orientation=[0, 0, -0.7, 0.7])
look_couch = Pose([3.8, 1.9, 0.8])
nav_pose1 = Pose([2, 1.3, 0], orientation=[0, 0, 0.4, 0.9])
greet_guest_pose = Pose(position=[1.7, 0.86, 0], orientation=[0, 0, 0.8, -0.5])

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)

image_switch_publisher = ImageSwitchPublisher()


def data_cb(data):
    """
    function to receive data from nlp via /nlp_out topic
    """
    global response
    global callback

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    for ele in response:
        ele.strip()
    response.append("None")
    print(response)
    callback = True


def welcome_guest(num, guest: HumanDescription):
    """
    talking sequence to get name and favorite drink of guest
    and attributes if it is the first guest
    :param num: number of guest
    :param guest: variable to store information in
    """
    global callback
    global wait_bool

    TalkingMotion("Welcome, please step in front of me and come close").perform()

    # look for human
    DetectAction(technique='human').resolve().perform()
    rospy.sleep(1)

    # look at guest and introduce
    HeadFollowMotion(state="start").perform()
    # giskardpy.move_head_to_human()
    rospy.sleep(2.3)

    TalkingMotion("Hello, i am Toya and my favorite drink is oil.").perform()
    rospy.sleep(3)

    TalkingMotion("What is your name and favorite drink?").perform()
    rospy.sleep(2.5)
    TalkingMotion("please answer me when my display changes").perform()
    rospy.sleep(2.5)

    # signal to start listening
    print("nlp start")
    pub_nlp.publish("start listening")
    rospy.sleep(2.3)
    image_switch_publisher.pub_now(ImageEnum.TALK.value)

    # wait for nlp answer
    start_time = time.time()
    while not callback:
        rospy.sleep(1)

        if int(time.time() - start_time) == timeout:
            print("guest needs to repeat")
            image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

    callback = False

    if response[0] == "<GUEST>":
        # success a name and intent was understood
        if response[1].strip() != "None" and response[2].strip() != "None":
            print("in success")
            # understood both
            guest.set_drink(response[2])
            guest.set_name(response[1])
        else:
            name = False
            drink = False
            if response[1].strip() == "None":
                # ask for name again once
                name = True
                guest.set_drink(response[2])

            if response[2].strip() == "None":
                drink = True
                # ask for drink again
                guest.set_name(response[1])

            if name:
                guest.set_name(name_repeat())

            if drink:
                guest.set_drink(drink_repeat())

    else:
        # two chances to get name and drink
        i = 0
        while i < 2:
            TalkingMotion("please repeat your name and drink loud and clear").perform()
            rospy.sleep(2.1)

            pub_nlp.publish("start")
            rospy.sleep(2.5)
            image_switch_publisher.pub_now(ImageEnum.TALK.value)

            start_time = time.time()
            while not callback:
                rospy.sleep(1)
                if int(time.time() - start_time) == timeout:
                    print("guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            callback = False

            if response[0] == "<GUEST>":
                # success a name and intent was understood
                if response[1].strip() != "None" and response[2].strip() != "None":
                    print("in success")
                    # understood both
                    guest.set_drink(response[2])
                    guest.set_name(response[1])
                    break
                else:
                    name = False
                    drink = False
                    if response[1].strip() == "None":
                        # ask for name again once
                        name = True
                        guest.set_drink(response[2])

                    if response[2].strip() == "None":
                        drink = True
                        # ask for drink again
                        guest.set_name(response[1])

                    if name:
                        guest.set_name(name_repeat())
                        break

                    if drink:
                        guest.set_drink(drink_repeat())
                        break

    # get attributes and face if first guest
    if num == 1:
        try:
            TalkingMotion("i will take a picture of you to recognize you later").perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            rospy.sleep(1.4)
            TalkingMotion("please wait and look at me").perform()
            config = {'arm_lift_joint': 0.05, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2,
                      'wrist_flex_joint': -1.5, 'wrist_roll_joint': 0, 'head_pan_joint': 0, 'head_tilt_joint': 1}

            get_attributes(guest)

        except PerceptionObjectNotFound:
            # failure handling, if human has stepped away
            TalkingMotion("please step in front of me").perform()
            rospy.sleep(3.5)

            try:
                get_attributes(guest)

            except PerceptionObjectNotFound:
                print("continue without attributes")

    TalkingMotion("i will show you the living room now").perform()

    print("guest ID welcome seq:" + str(guest.id))
    rospy.sleep(1.2)
    return guest


def detect_point_to_seat(no_sofa: Optional[bool] = False):
    """
    function to look for a place to sit and poit to it
    returns bool if free place found or not
    """

    # detect free seat
    seat = DetectAction(technique='location', state="sofa").resolve().perform()
    free_seat = False
    x = 3.6
    # loop through all seating options detected by perception
    if not no_sofa:
        print("in placequ")
        for place in seat:
            if place[1] == 'False':

                not_point_pose = Pose([float(place[2]), float(place[3]), 0.85])
                print("place: " + str(place))

                lt = LocalTransformer()
                pose_in_robot_frame = lt.transform_pose(not_point_pose, robot.get_link_tf_frame("base_link"))
                print("in robot: " + str(pose_in_robot_frame))
                if pose_in_robot_frame.pose.position.y > 0.25:
                    TalkingMotion("please take a seat to the left from me").perform()
                    pose_in_robot_frame.pose.position.y += 0.2
                    x = 4.9

                elif pose_in_robot_frame.pose.position.y < -0.35:
                    TalkingMotion("please take a seat to the right from me").perform()
                    pose_in_robot_frame.pose.position.y -= 0.4
                    x = 2.9

                else:
                    TalkingMotion("please take a seat in front of me").perform()

                pose_in_map = lt.transform_pose(pose_in_robot_frame, "map")
                free_seat = True
                break
    else:
        print("only chairs")
        for place in seat[1]:
            if place[0] == 'chair':
                if place[1] == 'False':
                    pose_in_map = Pose([float(place[2]), float(place[3]), 0.85])
                    TalkingMotion("please take a seat on the free chair").perform()
                    free_seat = True
                    x = 2.5
                    break

    if free_seat:
        pose_guest = PointStamped()
        pose_guest.header.frame_id = "map"
        pose_guest.point.x = x
        pose_guest.point.y = pose_in_map.pose.position.y
        pose_guest.point.z = 0.85

        PointingAction(x_coordinate=float(x), y_coordinate=float(pose_guest.point.y), z_coordinate=float(pose_guest.point.z)).resolve().perform()
        # giskardpy.move_arm_to_point(pose_guest)


        print("found seat")
        return pose_guest
    else:
        TalkingMotion("no free seat detected").perform()

    return free_seat




# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    # receive data from nlp via topic
    rospy.Subscriber("nlp_out", String, data_cb)
    # TalkingMotion("Starting demo").perform()
    rospy.sleep(2)
    #HeadFollowMotion(state="start").perform()
    rospy.sleep(3)
    x = detect_point_to_seat()
    print(x)
    #get_attributes(guest1)
    # welcome_guest(1, guest1)
    #print(guest1.name)
    # print(guest1.fav_drink)


