import rospy

from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from pycram.enums import ImageEnum as ImageEnum
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter

world = BulletWorld("DIRECT")
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change to right map
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_jule_1.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# important Publishers
move = PoseNavigator()
talk = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)
pub_bell = rospy.Publisher('/startSoundDetection', String, queue_size=16)
response = ""
wait_bool = False
callback = False
doorbell = False
timeout = 12  # 12 seconds timeout
laser = StartSignalWaiter()

# Declare variables for humans
# TODO: change to given name before challenge
host = HumanDescription("John", fav_drink="milk")

guest1 = HumanDescription("Lisa", fav_drink="water")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])

guest2 = HumanDescription("Sarah", fav_drink="Juice")
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

# pose variables
door_pose = Pose([3, 1.0, 0], [0, 0, 1, 0])
convo_pose = Pose([4.2, 0.15, 0], [0, 0, 1, 0])
convo_pose_to_couch = Pose([4.2, 0.15, 0], [0, 0, 0, 1])
couch_pose = Pose([8.5, 0, 0], [0, 0, 0, 1])
couch_pose_to_door = Pose([8.6, 0, 0], [0, 0, 1, 0])
start_pose = Pose([3.9, 0.2, 0], [0, 0, 1, 0])

# TODO: read from semantic map?
couch_pose_semantik = PoseStamped()
couch_pose_semantik.pose.position.x = 10.8
couch_pose_semantik.pose.position.y = -0.27
couch_pose_semantik.pose.position.z = 0.8


# reset giskard world
giskardpy.clear()
giskardpy.sync_worlds()


def data_cb(data):
    """
    function to receive data from nlp via /nlp_out topic
    """
    global response
    global callback
    global doorbell

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    for ele in response:
        ele.strip()
    response.append("None")
    print(response)
    callback = True


def doorbell_cb(data):
    """
    function to receive if doorbell was heard from nlp
    """
    global doorbell
    doorbell = True


def pakerino(torso_z=0.05, config=None):
    """
    replace function for park arms, robot takes pose of configuration of joint
    """

    if not config:
        config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2,
                  'wrist_flex_joint': -1.5, 'wrist_roll_joint': 0, 'head_pan_joint': 0}

    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("Parking done")


def door_opening():
    """
    door opening sequence
    """
    move.pub_now(door_pose)

    # TODO: check pre pose for door
    config = {'arm_lift_joint': 0.35, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
              'wrist_roll_joint': -1.57}

    pakerino(config=config)
    # move.pub_now(pre_pose_door)

    # grasp handle
    gripper.pub_now("open")
    giskardpy.grasp_doorhandle("kitchen_2/iai_kitchen:arena:door_handle_inside")
    gripper.pub_now("close")

    # open door
    talk.pub_now("open door now")
    giskardpy.open_doorhandle("kitchen_2/iai_kitchen:arena:door_handle_inside")
    gripper.pub_now("open")


    # TODO: test laser scanner and adjust threshhold
    door_open_bool = laser.something_in_the_way()
    if door_open_bool:
        talk.pub_now("I opened the door")
    else:
        talk.pub_now("I made a mistake. Please open the door yourself")

    # reset world state
    giskardpy.clear()
    giskardpy.sync_worlds()


def get_attributes(guest: HumanDescription):
    """
    storing attributes and face of person in front of robot
    :param guest: variable to store information in
    """
    # remember face
    keys = DetectAction(technique='human', state='face').resolve().perform()[1]
    new_id = keys["keys"][0]
    guest1.set_id(new_id)

    # get clothes and gender
    attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
    guest1.set_attributes(attr_list)
    rospy.loginfo(attr_list)


def name_repeat():
    """
    HRI-function to ask for name again once.
    """

    global callback
    global response
    global wait_bool
    callback = False
    got_name = False
    rospy.Subscriber("nlp_out", String, data_cb)

    while not got_name:
        talk.pub_now("i am sorry, please repeat your name", wait_bool=wait_bool)
        rospy.sleep(1.2)
        pub_nlp.publish("start")

        # sound/picture
        rospy.sleep(3)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not callback:
            # signal repeat to human
            if time.time() - start_time == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        image_switch_publisher.pub_now(ImageEnum.HI.value)
        callback = False

        if response[0] == "<GUEST>" and response[1].strip() != "None":
            return response[1]


def drink_repeat():
    """
    HRI-function to ask for drink again once.
    """
    global callback
    # global response
    global wait_bool
    callback = False
    got_name = False
    rospy.Subscriber("nlp_out", String, data_cb)

    while not got_name:
        talk.pub_now("i am sorry, please repeat your drink loud and clear", wait_bool=wait_bool)
        rospy.sleep(3.5)
        talk.pub_now("and use the sentence: my favorite drink is", wait_bool=wait_bool)
        rospy.sleep(3.3)
        pub_nlp.publish("start")

        # sound/picture
        rospy.sleep(3)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not callback:
            # signal repeat to human
            if time.time() - start_time == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        image_switch_publisher.pub_now(ImageEnum.HI.value)
        callback = False

        if response[0] == "<GUEST>" and response[2].strip() != "None":
            return response[2]


def welcome_guest(num, guest: HumanDescription):
    """
    talking sequence to get name and favorite drink of guest
    and attributes if it is the first guest
    :param num: number of guest
    :param guest: variable to store information in
    """
    global callback
    global wait_bool

    talk.pub_now("Welcome, please step in front of me and come close", wait_bool=wait_bool)

    # look for human
    DetectAction(technique='human').resolve().perform()
    rospy.sleep(1)

    # look at guest and introduce
    giskardpy.move_head_to_human()
    rospy.sleep(2.3)

    talk.pub_now("Hello, i am Toya and my favorite drink is oil.", True, wait_bool=wait_bool)
    rospy.sleep(3)

    talk.pub_now("What is your name and favorite drink?", True, wait_bool=wait_bool)
    rospy.sleep(2)

    talk.pub_now("please answer me after the beep sound", True, wait_bool=wait_bool)
    rospy.sleep(2.5)

    # signal to start listening
    rospy.loginfo("nlp start")
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
            elif response[2].strip() == "None":
                drink = True
                # ask for drink again
                guest.set_name(response[1])
            else:
                name = True
                drink = True

            if name:
                guest.set_name(name_repeat())

            if drink:
                guest.set_drink(drink_repeat())

    else:
        # two chances to get name and drink
        i = 0
        while i < 2:
            talk.pub_now("please repeat your name and drink loud and clear", True, wait_bool=wait_bool)
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
                    elif response[2].strip() == "None":
                        drink = True
                        # ask for drink again
                        guest.set_name(response[1])
                    else:
                        name = True
                        drink = True

                    if name:
                        guest.set_name(name_repeat())
                        break

                    if drink:
                        guest.set_drink(drink_repeat())
                        break

    # get attributes and face if first guest
    if num == 1:
        try:
            talk.pub_now("i will take a picture of you to recognize you later", wait_bool=wait_bool)
            pakerino()
            rospy.sleep(1.4)
            talk.pub_now("please wait and look at me", wait_bool=wait_bool)
            get_attributes(guest)

        except PerceptionObjectNotFound:
            # failure handling, if human has stepped away
            talk.pub_now("please step in front of me", wait_bool=wait_bool)
            rospy.sleep(3.5)

            try:
                get_attributes(guest)

            except PerceptionObjectNotFound:
                print("continue without attributes")

    talk.pub_now("i will show you the living room now", wait_bool=wait_bool)

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
    # loop through all seating options detected by perception
    if not no_sofa:
        for place in seat[1]:
            if place[1] == 'False':

                not_point_pose = Pose([float(place[2]), float(place[3]), 0.85])
                print("place: " + str(place))

                lt = LocalTransformer()
                pose_in_robot_frame = lt.transform_pose(not_point_pose, robot.get_link_tf_frame("base_link"))
                print("in robot: " + str(pose_in_robot_frame))
                # TODO: change according to new map and test lol
                if pose_in_robot_frame.pose.position.y > 0.15:
                    talk.pub_now("please take a seat to the left from me")
                    pose_in_robot_frame.pose.position.y += 0.2

                elif pose_in_robot_frame.pose.position.y < -0.45:
                    talk.pub_now("please take a seat to the right from me")
                    pose_in_robot_frame.pose.position.y -= 0.8

                else:
                    talk.pub_now("please take a seat in front of me")

                pose_in_map = lt.transform_pose(pose_in_robot_frame, "map")
                free_seat = True
                break
    else:
        print("only chairs")
        for place in seat[1]:
            if place[0] == 'chair':
                if place[1] == 'False':
                    pose_in_map = Pose([float(place[2]), float(place[3]), 0.85])
                    talk.pub_now("please take a seat on the free chair")
                    free_seat = True

    if free_seat:
        pose_guest = PointStamped()
        pose_guest.header.frame_id = "map"
        pose_guest.point.x = pose_in_map.pose.position.x
        pose_guest.point.y = pose_in_map.pose.position.y
        pose_guest.point.z = 0.85
        print(pose_guest)

        giskardpy.move_arm_to_point(pose_guest)

        print("found seat")
        return pose_guest
    return free_seat


def demo(step):
    with real_robot:
        global wait_bool
        global callback
        global doorbell
        global guest1
        global guest2
        pose3 = Pose([1.9, 4.5, 0], [0, 0, 1, 0])

        # signal start
        pakerino()

        talk.pub_now("waiting for guests", wait_bool=wait_bool)
        image_switch_publisher.pub_now(ImageEnum.HI.value)

        # receive data from nlp via topic
        rospy.Subscriber("nlp_out", String, data_cb)
        rospy.Subscriber("nlp_out2", String, doorbell_cb)

        if step <= 0:
            # door opening sequence
            pub_bell.publish("start bell detection")

            # wait 15 seconds for sound
            start_time = time.time()
            while not doorbell:
                # continue challenge to not waste time
                if time.time() - start_time > timeout:
                    print("Timeout reached, no bell")
                    break

                time.sleep(0.5)

            # set it back to false for second guest
            doorbell = False

            # door_opening()

        if step <= 1:
            # reception/talking sequence
            pakerino()

            # move away from door
            move.pub_now(convo_pose)

            # HRI sequence
            guest1 = welcome_guest(1, guest1)

        if step <= 2:
            # stop looking at human
            giskardpy.cancel_all_called_goals()
            DetectAction(technique='human', state='stop').resolve().perform()

            # leading to living room and pointing to free seat
            talk.pub_now("follow me from the hallway to the living room", wait_bool=wait_bool)
            rospy.sleep(2)
            talk.pub_now("please step out of my way and stay behind me", wait_bool=wait_bool)
            pakerino()

            # head straight
            pakerino()
            rospy.sleep(0.5)

            # lead human to living room
            move.pub_now(convo_pose_to_couch, interrupt_bool=False)
            move.pub_now(couch_pose)

        if step <= 3:
            # place new guest in living room
            gripper.pub_now("close")
            talk.pub_now("welcome to the living room", wait_bool=wait_bool)
            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)

            # update poses from guest1 and host
            counter = 0
            while counter < 6:
                found_host = False
                try:
                    human_dict = DetectAction(technique='human', state='face').resolve().perform()[1]
                    counter += 1
                    id_humans = human_dict["keys"]
                    print("id humans: " + str(id_humans))
                    host_pose = human_dict[id_humans[0]]
                    host.set_id(id_humans[0])
                    host.set_pose(PoseStamped_to_Point(host_pose))
                    found_host = True
                    break

                except:
                    print("i am a faliure and i hate my life")
                    if counter == 2:
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(1.5)

                    elif counter == 3:
                        # TODO change according to arena
                        config = {'head_pan_joint': -0.35}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(1.5)

                    elif counter == 5:
                        # TODO change according to arena
                        config = {'head_pan_joint': 0.25}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(1.5)

                    counter += 1

            if not found_host:
                try:
                    rospy.logerr("not found host human detect")
                    pakerino()
                    host_pose = DetectAction(technique='human').resolve().perform()[1]
                    host.set_pose(host_pose)

                except Exception as e:
                    print(e)

            # find free seat
            giskardpy.move_head_to_pose(couch_pose_semantik)
            guest_pose = detect_point_to_seat()
            image_switch_publisher.pub_now(ImageEnum.SOFA.value)
            if not guest_pose:
                # move head a little to perceive chairs
                # TODO: change according to arena
                config = {'head_pan_joint': -0.35}
                pakerino(config=config)
                guest_pose = detect_point_to_seat(no_sofa=True)
                # turn left
                if not guest_pose:
                    config = {'head_pan_joint': 0.15}
                    pakerino(config=config)
                    guest_pose = detect_point_to_seat(no_sofa=True)
                    guest1.set_pose(guest_pose)
                else:
                    guest1.set_pose(guest_pose)
            else:
                guest1.set_pose(guest_pose)

        if step <= 4:
            # introduce guest1 and host
            giskardpy.move_head_to_human()
            rospy.sleep(1)
            if guest1.pose:
                pub_pose.publish(guest1.pose)

            rospy.sleep(1.5)

            # introduce humans and look at them
            introduce(host, guest1)
            rospy.sleep(2)
            giskardpy.cancel_all_called_goals()

        if step <= 5:

            # drive back to door
            # head straight
            # TODO: change accoring to map
            pakerino()
            rospy.sleep(0.5)

            # drive back to hallway
            move.pub_now(couch_pose_to_door, interrupt_bool=False)
            move.pub_now(start_pose)

            # signal start
            talk.pub_now("waiting for new guests", wait_bool=wait_bool)
            image_switch_publisher.pub_now(ImageEnum.HI.value)

            # wait 12 seconds for sound
            pub_bell.publish("start bell detection")
            start_time = time.time()
            while not doorbell:
                # continue challenge to not waste time
                if time.time() - start_time > timeout:
                    print("Timeout reached, no bell")
                    break

                time.sleep(0.5)

            # door_opening()
            pakerino()
            rospy.sleep(1)

        if step <= 6:

            move.pub_now(convo_pose)
            guest2 = welcome_guest(2, guest2)

        if step <= 7:

            # leading to living room and pointing to free seat
            talk.pub_now("follow me from the hallway to the living room", wait_bool=wait_bool)
            rospy.sleep(2)
            talk.pub_now("please step out of my way and stay behind me", wait_bool=wait_bool)
            pakerino()

            # stop looking at human
            giskardpy.cancel_all_called_goals()
            DetectAction(technique='human', state='stop').resolve().perform()

            # lead human to living room
            # TODO: change according to arena
            move.pub_now(convo_pose_to_couch, interrupt_bool=False)
            move.pub_now(couch_pose)

            talk.pub_now("welcome to the living room", wait_bool=wait_bool)

            # place new guest in living room
        if step <= 8:

            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)
            gripper.pub_now("close")

            # 3 trys to update poses from guest1 and host
            counter = 0
            found_guest = False
            found_host = False
            while True:
                unknown = []
                try:
                    human_dict = DetectAction(technique='human', state='face').resolve().perform()[1]
                    counter += 1
                    id_humans = human_dict["keys"]

                    # loop through detected face Ids
                    for key in id_humans:
                        print("key: " + str(key))
                        print("counter: " + str(counter))

                        # found guest
                        if key == guest1.id:
                            # update pose
                            guest1_pose = human_dict[guest1.id]
                            found_guest = True
                            guest1.set_pose(PoseStamped_to_Point(guest1_pose))

                        # found host
                        elif key == host.id:
                            # update pose
                            found_host = True
                            host_pose = human_dict[host.id]
                            host.set_pose(PoseStamped_to_Point(host_pose))
                        else:
                            # store unknown ids for failure handling
                            unknown.append(key)

                    if counter > 4 or (found_guest and found_host):
                        break

                    elif counter == 2:
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)
                    # move head to side to detect faces that were not in vision before
                    # TODO: change direction according to arena
                    elif counter == 3:
                        config = {'head_pan_joint': -0.3}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)

                except PerceptionObjectNotFound:
                    print("i am a failure and i hate my life")
                    counter += 1
                    print(counter)
                    if counter == 3:
                        config = {'head_pan_joint': -0.3}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)

            # Failure Handling if at least one person was not recognized
            if not found_guest and not found_host:
                try:
                    # both have not been recognized
                    guest1.set_pose(PoseStamped_to_Point(human_dict[unknown[0]]))
                    host.set_pose(PoseStamped_to_Point(human_dict[unknown[1]]))
                except Exception as e:
                    print(e)
            else:
                # either guest or host was not found
                if not found_guest:
                    try:
                        guest1.set_pose(PoseStamped_to_Point(human_dict[unknown[0]]))
                    except Exception as e:
                        print(e)
                elif not found_host:
                    try:
                        host.set_pose(PoseStamped_to_Point(human_dict[unknown[0]]))
                    except Exception as e:
                        print(e)

        if step <= 9:
            # look t couch
            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)

            # find a place for guest2 to sit and point
            guest_pose = detect_point_to_seat()
            image_switch_publisher.pub_now(ImageEnum.SOFA.value)

            if not guest_pose:
                # move head a little
                # TODO: change according to arena
                config = {'head_pan_joint': -0.3}
                pakerino(config=config)
                guest_pose = detect_point_to_seat(no_sofa=True)
                if guest_pose:
                    guest2.set_pose(guest_pose)
            else:
                guest2.set_pose(guest_pose)

            if step <= 10:
                # introduce everyone to guest 2
                giskardpy.move_head_to_human()
                rospy.sleep(1.2)
                introduce(host, guest2)
                rospy.sleep(3)
                introduce(guest1, guest2)
                rospy.sleep(3)
                describe(guest1)

# door_opening()
# demo(0)


# hri sequence test
# move.pub_now(convo_pose)
print(welcome_guest(1, guest1))

# from hallway to living room
# move.pub_now(convo_pose_to_couch, interrupt_bool=False)
# move.pub_now(couch_pose)
# talk.pub_now("welcome to the living room", wait_bool=wait_bool)

# step 3 finde host and free seat
# -> test with no host in living room
# -> test with people in background
# -> test with couch not free
# demo(3)
