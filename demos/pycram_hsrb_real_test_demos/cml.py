#!/usr/bin/env python3
import time

import actionlib
import rospy
from geometry_msgs.msg import PointStamped
from robokudo_msgs.msg import QueryAction, QueryGoal

import pycram.external_interfaces.giskard_new as giskardpy
from pycram.designators.action_designator import fts
from pycram.enums import ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.plan_failures import SensorMonitoringCondition, HumanNotFoundCondition
from pycram.process_module import real_robot
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, HSRBMoveGripperReal, pakerino

# Initialize the necessary components
# world = BulletWorld("DIRECT")
# v = VizMarkerPublisher()

# world = BulletWorld()
# v = VizMarkerPublisher()
# kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")

talk = TextToSpeechPublisher()
img = ImageSwitchPublisher()
gripper = HSRBMoveGripperReal()

# robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
# robot.set_color([0.5, 0.5, 0.9, 1])

# RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
# KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
rospy.loginfo("Loading done")
giskardpy.init_giskard_interface()
rkclient = actionlib.SimpleActionClient('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
rkclient.wait_for_server()
rospy.loginfo("You can start your demo now")

# Initialize global variable
global human_bool
human_bool = False


class Human:
    """
    Class that represents humans. This class does not spawn a human in a simulation.
    """

    def __init__(self):
        self.human_pose = Fluent()

        self.last_msg_time = time.time()

        self.threshold = 5.0  # seconds

        # Subscriber to the human pose topic
        self.human_pose_sub = rospy.Subscriber("/cml_human_pose", PointStamped, self.human_pose_cb)

        # Timer to check for no message
        self.timer = rospy.Timer(rospy.Duration(1), self.check_for_no_message)

    def check_for_no_message(self, event):
        current_time = time.time()
        if (current_time - self.last_msg_time) > self.threshold:
            # rospy.loginfo("No messages received for %s seconds", self.threshold)
            self.human_pose.set_value(False)

    def human_pose_cb(self, HumanPoseMsg):
        """
        Callback function for human_pose Subscriber.
        Sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic.
        :param HumanPoseMsg: received message
        """
        self.last_msg_time = time.time()

        if HumanPoseMsg:
            self.human_pose.set_value(True)
        else:
            self.human_pose.set_value(False)


# one_time_pub = rospy.Publisher("/cml_human_pose", PointStamped, queue_size=10)
# msg = PointStamped()
# one_time_pub.publish(msg)
human = Human()


def monitor_func_human():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        print("sensor")
        return SensorMonitoringCondition
    if not human.human_pose.get_value():
        print("human")
        return HumanNotFoundCondition
    return False


def monitor_func():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def callback(point):
    global human_bool
    human_bool = True
    print("Human detected")


def first_part(talk_bool):
    try:
        talk.pub_now("Starting Carry my Luggage demo.", talk_bool, False)
        img.pub_now(ImageEnum.HI.value)  # hi im toya
        talk.pub_now("Push down my Hand, when you are Ready.", talk_bool,False)
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        img.pub_now(ImageEnum.HI.value)  # hi im toya

        talk.pub_now("Looking for a human", talk_bool,False)
        img.pub_now(ImageEnum.SEARCH.value)  # search
        goal_msg = QueryGoal()
        rkclient.send_goal(goal_msg)
        # worksme this worked before i moved it
        rospy.loginfo("Waiting for human to be detected")
        no_human = True
        while no_human:

            if human.human_pose.wait_for(timeout=5):
                no_human = False
            else:
                talk.pub_now("Looking for a human, please step in front of me", talk_bool,False)
                rkclient.send_goal(goal_msg)

        talk.pub_now("Found a Human", talk_bool,False)
        img.pub_now(ImageEnum.HI.value)

        gripper.pub_now("open")
        rospy.sleep(1)
        talk.pub_now("I am not able to pick up the bag. Please hand it in", talk_bool,False)
        rospy.sleep(1)

        talk.pub_now("Push down my Hand, when you are Ready.", talk_bool,False)
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        try:
            plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            talk.pub_now("Closing my Gripper.", talk_bool)
            img.pub_now(ImageEnum.HI.value)
            gripper.pub_now("close")



def cml(step="default"):  # worksme

    with real_robot:
        talk_bool = True
        if step == "default":
            pakerino()
            first_part(talk_bool)
        if step == "lost_human":
            no_human = True
            while no_human:
                goal_msg = QueryGoal()
                if human.human_pose.get_value():
                    no_human = False
                else:
                    rospy.sleep(5)
                    talk.pub_now("I lost you, please step in front of me", talk_bool,False)
                    rkclient.send_goal(goal_msg)
        talk.pub_now("Following you.", talk_bool,False)
        img.pub_now(ImageEnum.FOLLOWSTOP.value)
        talk.pub_now("Push down my Hand, when we arrived.", talk_bool,False)

        try:
            plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func_human)
            plan.perform()
        except Exception as e:
            print(f"Exception type: {type(e).__name__}")

            if isinstance(e, SensorMonitoringCondition):

                talk.pub_now("We have arrived.", talk_bool,False)
                talk.pub_now("I will open my Gripper, to give you the bag.", talk_bool,False)
                talk.pub_now("Push down my Hand, when you are Ready.", talk_bool,False)
                img.pub_now(ImageEnum.PUSHBUTTONS.value)
                try:
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                    plan.perform()
                except SensorMonitoringCondition:
                    gripper.pub_now("open")
                    talk.pub_now("Driving Back.", talk_bool,False)
                    img.pub_now(ImageEnum.DRIVINGBACK.value)
                    giskardpy.cml(True)
                    talk.pub_now("done.", talk_bool,False)
            elif isinstance(e, HumanNotFoundCondition):
                cml("lost_human")
            # fixme i dont know what to do here
            else:
                print("idk what happned")


cml("default")

