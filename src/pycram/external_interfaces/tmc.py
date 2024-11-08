import rospy
from actionlib_msgs.msg import GoalStatusArray
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from tmc_msgs.msg import Voice, TalkRequestActionGoal
from typing_extensions import Optional

from ..datastructures.enums import GripperState
from ..designators.motion_designator import MoveGripperMotion, TalkingMotion
from ..ros.logging import loginfo, logwarn
from ..ros.publisher import create_publisher
from ..ros.data_types import Rate

is_init = False


def init_tmc_interface():
    global is_init
    if is_init:
        return
    try:
        from tmc_control_msgs.msg import GripperApplyEffortActionGoal
        from tmc_msgs.msg import Voice
        is_init = True
        loginfo("Successfully initialized tmc interface")
    except ModuleNotFoundError as e:
        logwarn(f"Could not import TMC messages, tmc interface could not be initialized")


def tmc_gripper_control(designator: MoveGripperMotion, topic_name: Optional[str] = '/hsrb/gripper_controller/grasp/goal'):
    """
    Publishes a message to the gripper controller to open or close the gripper for the HSR.

    :param designator: The designator containing the motion to be executed
    :param topic_name: The topic name to publish the message to
    """
    if (designator.motion == GripperState.OPEN):
        pub_gripper = create_publisher(topic_name, GripperApplyEffortActionGoal, 10)
        rate = Rate(10)
        msg = GripperApplyEffortActionGoal()
        msg.goal.effort = 0.8
        pub_gripper.publish(msg)

    elif (designator.motion == GripperState.CLOSE):
        pub_gripper = create_publisher(topic_name, GripperApplyEffortActionGoal, 10)
        rate = Rate(10)
        msg = GripperApplyEffortActionGoal()
        msg.goal.effort = -0.8
        pub_gripper.publish(msg)


class TextToSpeechPublisher:

    def __init__(self):
        self.pub = rospy.Publisher('/talk_request_action/goal', TalkRequestActionGoal, queue_size=10)
        self.status_sub = rospy.Subscriber('/talk_request_action/status', GoalStatusArray, self.status_callback)
        self.status_list = []

    def status_callback(self, msg):
        self.status_list = msg.status_list

    def pub_now(self, sentence, talk_bool: bool = True, wait_bool: bool = True):

        rospy.logerr("talking sentence: " + str(sentence))
        if talk_bool:
            while not rospy.is_shutdown():
                if not self.status_list or not wait_bool:  # Check if the status list is empty
                    goal_msg = TalkRequestActionGoal()
                    goal_msg.header.stamp = rospy.Time.now()
                    goal_msg.goal.data.language = 1
                    goal_msg.goal.data.sentence = sentence

                    while self.pub.get_num_connections() == 0:
                        rospy.sleep(0.1)

                    self.pub.publish(goal_msg)
                    break


def tmc_talk(designator: TalkingMotion):
    """
    Publishes a sentence to the talk_request topic of the HSRB robot

    :param designator: The designator containing the sentence to be spoken
    """
    talk = TextToSpeechPublisher()
    talk.pub_now(designator.cmd)
