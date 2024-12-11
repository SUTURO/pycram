from std_msgs.msg import String

from demos.pycram_receptionist_demo.utils.NLP_Functions import NLP_Functions
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
nlp = NLP_Functions()

# Declare variables for humans
host = HumanDescription("Bob", fav_drink="Milk")
host.set_id(1)

guest1 = HumanDescription("Lisa", fav_drink="water")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

# important poses
couch_pose_semantik = Pose(position=[3.8, 2.1, 0], orientation=[0, 0, -0.7, 0.7])
look_couch = Pose([3.8, 0.3, 0.75])
nav_pose1 = Pose([2, 1.3, 0], orientation=[0, 0, 0.4, 0.9])
greet_guest_pose = Pose(position=[1.9, 0.6, 0], orientation=[0, 0, 0.8, -0.5])


def demo(step: int):
    with (real_robot):
        rospy.loginfo("start demo at step " + str(step))

        # set neutral pose
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()

        if step <= 1:
            # greet first guest
            nlp.welcome_guest(guest1)

        if step <= 2:
            # perceive attributes of guest
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            get_attributes(guest1)

        if step <= 3:
            # lead to living room
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            TalkingMotion("please step out of the way and follow me").perform()
            NavigateAction([couch_pose_semantik]).resolve().perform()

        if step <= 4:
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
                    TalkingMotion("please look at me").perform()
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

        if step <= 5:
            # find free place to sit for guest
            LookAtAction([look_couch]).resolve().perform()
            guest_pose = detect_point_to_seat(robot)
            if not guest_pose:
                # look to the side to find seat
                MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                guest1.set_pose(guest_pose)
            else:
                guest1.set_pose(guest_pose)

        if step <= 6:
            # introduce sitting people
            HeadFollowMotion(state="start").perform()
            rospy.sleep(2)
            introduce(host, guest1)
            rospy.sleep(2)
            HeadFollowMotion(state="stop").perform()
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

        if step <= 7:
            # go back to start-pose
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            NavigateAction([greet_guest_pose]).resolve().perform()
            TalkingMotion("waiting for new guest").perform()
            image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 8:
            # greet second guest and lead to living room
            nlp.welcome_guest(guest2)
            MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            TalkingMotion("please step out of the way and follow me").perform()
            NavigateAction([couch_pose_semantik]).resolve().perform()

        if step <= 9:
            # search for host and guest
            TalkingMotion("welcome to the living room").perform()
            identify_faces(host, guest1)

        if step <= 10:
            # find free place for second guest
            LookAtAction([look_couch]).resolve().perform()
            guest_pose = detect_point_to_seat(robot)
            if not guest_pose:
                MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True)
                guest2.set_pose(guest_pose)
            else:
                guest2.set_pose(guest_pose)

        if step <= 11:
            # introduce everyone and state attributes of first guest
            HeadFollowMotion(state="start").perform()
            rospy.sleep(1.5)
            introduce(host, guest2)
            rospy.sleep(3)
            introduce(guest1, guest2)
            rospy.sleep(3)
            describe(guest1)
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()


demo(0)
