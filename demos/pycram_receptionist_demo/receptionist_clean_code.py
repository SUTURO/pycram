from typing import Optional

import rospy

from demos.pycram_receptionist_demo.utils.NLP_Functions import NLP_Functions
from demos.pycram_receptionist_demo.utils.helper import *

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

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")

# Define orientation for objects
# object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

response = [None, None, None]
callback = False

# Declare variables for humans
host = HumanDescription("Bob", fav_drink="Milk")
host.set_id(1)
guest1 = HumanDescription("Lisa", fav_drink="water")

# for testing, if the first part of the demo is skipped
#guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
# for testing, if the first part of the demo is skipped
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

# important poses
couch_pose_semantik = Pose(position=[3.8, 2.1, 0], orientation=[0, 0, -0.7, 0.7])
look_couch = Pose([3.8, 1.9, 0.8])
nav_pose1 = Pose([2, 1.3, 0], orientation=[0, 0, 0.4, 0.9])
greet_guest_pose = Pose(position=[2.2, 0.6, 0], orientation=[0, 0, 0.8, -0.5])

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)

image_switch_publisher = ImageSwitchPublisher()

nlp = NLP_Functions()


def demo(step: int):
    with (real_robot):
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        TalkingMotion("start demo").perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveJointsMotion(["torso_lift_joint"], [0.0]).perform()

        if step <= 1:
            nlp.welcome_guest(guest1)

        if step <= 2:
            get_attributes(guest1)
            print("guest 1 id: " + str(guest1.id))

        if step <= 3:
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            TalkingMotion("please step out of the way and follow me").perform()
            NavigateAction([couch_pose_semantik]).resolve().perform()
        if step <= 4:
            TalkingMotion("welcome to the living room").perform()
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
                    MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                    TalkingMotion("please look at me").perform()
                    rospy.sleep(1.5)

                if counter == 5:
                    try:
                        print("host has no id")
                        host_pose = DetectAction(technique='human').resolve().perform()
                        host.set_pose(host_pose)

                    except Exception as e:
                        print(e)
                    break

                counter += 1

        if step <= 5:
            guest_pose = detect_point_to_seat(robot)
            if not guest_pose:
                MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True, robot=robot)
                guest1.set_pose(guest_pose)
            else:
                guest1.set_pose(guest_pose)

        if step <= 6:
            HeadFollowMotion(state="start").perform()
            rospy.sleep(2)
            introduce(host, guest1)
            rospy.sleep(2)
            HeadFollowMotion(state="stop").perform()
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

        if step <= 7:
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            NavigateAction([greet_guest_pose]).resolve().perform()
            TalkingMotion("waiting for new guest").perform()
            image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 8:
            nlp.welcome_guest(guest2)
            TalkingMotion("i will show you the living room now").perform()
            rospy.sleep(1.5)
            TalkingMotion("please step out of the way and follow me").perform()
            NavigateAction([couch_pose_semantik]).resolve().perform()

        if step <= 9:
            TalkingMotion("Welcome to the living room").perform()
            identify_faces(host, guest1)

        if step <= 10:
            guest_pose = detect_point_to_seat(robot)
            if not guest_pose:
                MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                guest_pose = detect_point_to_seat(no_sofa=True)
                guest2.set_pose(guest_pose)
            else:
                guest2.set_pose(guest_pose)

        if step <= 11:
            HeadFollowMotion(state="start").perform()
            rospy.sleep(1.5)
            introduce(host, guest2)
            rospy.sleep(3)
            introduce(guest1, guest2)
            rospy.sleep(3)
            describe(guest1)
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()


demo(0)
