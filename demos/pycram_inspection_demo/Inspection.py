import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.action_designator import NavigateAction
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.failures import SensorMonitoringCondition
from pycram.language import Code, Monitor
from pycram.process_module import real_robot
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycram.utilities.robocup_utils import StartSignalWaiter, ImageSwitchPublisher
import rospy
from pycram.external_interfaces.navigate import PoseNavigator

# Class that automatically senses if an object (like a door) is in the way
start_signal = StartSignalWaiter()

# Class to get value, if gripper was pushed down
fts = ForceTorqueSensor(robot_name='hsrb')

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
RobotStateUpdater("/tf", "/giskard_joint_states")

# change HSR diplay pictures
img = ImageSwitchPublisher()

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")
pose1 = Pose(position=[1.7, -1, 0], orientation=[0, 0, -1, 0]) # change Poses!
pose2 = Pose(position=[2, 0, 0], orientation=[0, 0, 0.7, 0.7])
inspection_pose = Pose(position=[2, 0, 0], orientation=[0, 0, 0.7, 0.7])
exit_pose = Pose(position=[2, 0, 0], orientation=[0, 0, 0.7, 0.7])

# interface class for navigation and localization
navigation = PoseNavigator()


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        rospy.logwarn("sensor exception")
        return SensorMonitoringCondition

    return False


def inspection():
    with real_robot:
        img.pub_now(ImageEnum.HI.value)
        MoveJointsMotion(["arm_roll_joint"], [-1.2]).perform()
        TalkingMotion("push down my gripper").perform()
        try:
            plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
            plan.perform()

        except SensorMonitoringCondition:
            TalkingMotion("please open the door to let me in").perform()
            start_signal.wait_for_startsignal()

            # received start signal - localize robot again
            navigation.pub_fake_pose(robot.get_pose())
            giskardpy.turning_left_and_back(45)

            # start navigaon
            NavigateAction([pose1]).resolve().perform()
            TalkingMotion("on my way to the inspection point").perform()
            NavigateAction([pose2]).resolve().perform()
            NavigateAction([inspection_pose]).resolve().perform()
            TalkingMotion("I reached the inspection point").perform()
            NavigateAction([exit_pose]).resolve().perform()


inspection()
