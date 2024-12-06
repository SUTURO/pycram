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

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")

# Define orientation for objects
# object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

response = [None, None, None]
callback = False

# Declare variables for humans
host = HumanDescription("Vanessa", fav_drink="coffee")
host.set_id(1)
guest1 = HumanDescription("Lisa", fav_drink="water")

# for testing, if the first part of the demo is skipped
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
# for testing, if the first part of the demo is skipped
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

# important poses
couch_pose_semantik = Pose(position=[3.8, 2.1, 0], orientation=[0, 0, -0.7, 0.7])
look_couch = Pose([3.8, 0.3, 0.8])
nav_pose1 = Pose([2, 1.3, 0], orientation=[0, 0, 0.4, 0.9])
greet_guest_pose = Pose(position=[2.2, 1, 0], orientation=[0, 0, 0.8, -0.5])

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)

image_switch_publisher = ImageSwitchPublisher()

nlp = NLP_Functions()


def demo(step: int):
    with (real_robot):
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        ParkArmsAction([Arms.LEFT]).resolve().perform()

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
            LookAtAction([look_couch]).resolve().perform()
            TalkingMotion("please take da seat on the couch").perform()
            pose_guest = PointStamped()
            pose_guest.header.frame_id = "map"
            pose_guest.point.x = 3.7
            pose_guest.point.y = 0.3
            pose_guest.point.z = 0.85
            guest1.set_pose(pose_guest)

            MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
            PointingMotion(pose_guest).perform()

        if step <= 6:
            HeadFollowMotion(state="start").perform()
            rospy.sleep(2)
            introduce(host, guest1)
            rospy.sleep(2)
            HeadFollowMotion(state="stop").perform()
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            describe(guest1)

        if step <= 7:
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            NavigateAction([greet_guest_pose]).resolve().perform()

demo(0)
