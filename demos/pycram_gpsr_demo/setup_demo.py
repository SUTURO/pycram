# setup the environment
from pycram.designators.action_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.pose import Pose
from pycram.process_module import simulated_robot, real_robot
from pycram.robot_descriptions import robot_description
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import ImageSwitchPublisher, SoundRequestPublisher
from . import utils, high_level_plans, knowrob_interface

# import pycram.external_interfaces.giskard_new as giskard
import tf

# initialize interfaces
world = 0
robot = 0
environment = 0
rviz = 0
instruction_point = Pose([1.45, 4.5, 0], [0, 0, 1, 0])
move = PoseNavigator()
image_switch = ImageSwitchPublisher()
sound_pub = SoundRequestPublisher()
#kb = KnowrobKnowledge()
tf_listener = tf.listener.TransformListener()
with_real_robot = False


# maybe move this into the setup function so that it doesn't get auto-executed?
# init demo in repl:  import demos.pycram_gpsr_demo as gpsr


def setup():
    with (real_robot):
        rospy.loginfo("init")
        global world, robot, environment, rviz, plan_list, giskard, move, tts, image_switch, sound_pub
        global tf_listener, with_real_robot
        # giskard.init_giskard_interface()
        # giskard.sync_worlds()
        # world = BulletWorld("DIRECT") #rviz only, without any parameters, spawns bullet
        knowrob_interface.init_knowrob()
        rospy.loginfo("init setup")
        world = BulletWorld("DIRECT")
        rviz = VizMarkerPublisher()

        robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
        robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
        robot.set_color([0.5, 0.0, 0.2, 1])

        environment = Object("environment", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
        environment_desig = ObjectDesignatorDescription(names=["environment"])

        # sync to kitchen and robot
        RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
        # KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
        # giskard.sync_worlds()
        with_real_robot = True
        rospy.loginfo("done with setup")


def do_stuff():
    with simulated_robot:
        rospy.loginfo("now doing things")
        pose1 = Pose([1.45, 4.5, 0], [0, 0, 1, 0])
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        NavigateAction([pose1]).resolve().perform()
        MoveJointsMotion(["wrist_roll_joint"], [-1.57]).resolve().perform()
        MoveTorsoAction([0.35]).resolve().perform()
        MoveGripperMotion(motion="open", gripper="left").resolve().perform() #fails
        rospy.loginfo("done")


def test():
    global plan_list
    plan_list = utils.get_plans(high_level_plans)
    rospy.loginfo("imported all plans: ")
    rospy.loginfo(plan_list)
    rospy.loginfo("attempt to call cleaning with param test")
    utils.call_plan_by_name(plan_list, "cleaning", "test")
    # rviz marker publisher
    marker = ManualMarkerPublisher()
    tp = Pose(frame='map', position=[1, 1, 0], orientation=[1, 0, 0, 0])
    marker.publish(tp)


# setup() only if launching via pycharm
# import importlib
# importlib.reload(gpsr)