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
#host.set_id(0)

guest1 = HumanDescription("Lisa", fav_drink="water")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
#guest1.set_id(2)

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
                    pose_in_robot_frame.pose.position.y -= 0.55

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
                    break

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

        # signal start
        pakerino()
        if step <= 2:

            # update poses from guest1 and host
            #move.pub_now(couch_pose)
            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)
            talk.pub_now("detecting host an free seat", wait_bool=wait_bool)

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
                    print("i am a failure and i hate my life")
                    if counter == 2:
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(1.5)

                    elif counter == 3:
                        config = {'head_pan_joint': -0.75}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(1.5)

                    elif counter == 5:
                        config = {'head_pan_joint': 0.4}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(1.5)

                    counter += 1

            if not found_host:
                try:
                    rospy.logerr("no host now human detect")
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
                # move head to right to perceive chairs
                config = {'head_pan_joint': -0.75}
                pakerino(config=config)
                guest_pose = detect_point_to_seat(no_sofa=True)
                # turn left
                if not guest_pose:
                    config = {'head_pan_joint': 0.4}
                    pakerino(config=config)
                    guest_pose = detect_point_to_seat(no_sofa=True)
                    guest1.set_pose(guest_pose)
                else:
                    guest1.set_pose(guest_pose)
            else:
                guest1.set_pose(guest_pose)

        if step <= 3:
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

        if step <= 4:
            # place new guest in living room
            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)
            talk.pub_now("detecting host and guest and seat", wait_bool=wait_bool)
            gripper.pub_now("close")

            # 6 trys to update poses from guest1 and host
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

                    if counter > 6 or (found_guest and found_host):
                        break

                    elif counter == 2:
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)
                    # move head to side to detect faces that were not in vision before
                    elif counter == 3:
                        config = {'head_pan_joint': -0.85}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)
                    elif counter == 5:
                        config = {'head_pan_joint': 0.4}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)

                except PerceptionObjectNotFound:
                    print("i am a failure and i hate my life")
                    counter += 1
                    print(counter)
                    if counter == 3:
                        config = {'head_pan_joint': -0.85}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)
                    elif counter == 5:
                        config = {'head_pan_joint': 0.4}
                        pakerino(config=config)
                        talk.pub_now("please look at me", wait_bool=wait_bool)
                        rospy.sleep(2.5)

                    if counter > 6 or (found_guest and found_host):
                        break

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

        if step <= 5:
            # look to couch
            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)

            # find a place for guest2 to sit and point
            guest_pose = detect_point_to_seat()
            image_switch_publisher.pub_now(ImageEnum.SOFA.value)

            if not guest_pose:
                # move head a little to right
                # TODO: change according to arena
                config = {'head_pan_joint': -0.75}
                pakerino(config=config)
                guest_pose = detect_point_to_seat(no_sofa=True)
                if not guest_pose:
                    # move head a little to left
                    # TODO: change according to arena
                    config = {'head_pan_joint': 0.4}
                    pakerino(config=config)
                    guest_pose = detect_point_to_seat(no_sofa=True)
                    guest2.set_pose(guest_pose)
                else:
                    guest2.set_pose(guest_pose)
            else:
                guest2.set_pose(guest_pose)

            if step <= 6:
                # introduce everyone to guest 2
                giskardpy.move_head_to_human()
                rospy.sleep(1.2)
                introduce(host, guest2)
                rospy.sleep(3)
                introduce(guest1, guest2)
                rospy.sleep(3)
                describe(guest1)

demo(0)