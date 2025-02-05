import time
import rospy
from geometry_msgs.msg import PointStamped
from pycram.fluent import Fluent


class Human:
    """
    Class that represents humans. This class does not spawn a human in a simulation.
    """

    def __init__(self):
        self.human_pose = Fluent()

        self.last_msg_time = time.time()

        self.threshold = 3.0  # seconds

        # Subscriber to the human pose topic
        #TODO check if its really pointstamped (it should)
        self.human_pose_sub = rospy.Subscriber("/human_pose", PointStamped, self.human_pose_cb)

        # Timer to check for no message
        self.timer = rospy.Timer(period=rospy.Duration(1), callback=self.check_for_no_message)

    def check_for_no_message(self):
        current_time = time.time()
        print("Timer")
        if (current_time - self.last_msg_time) > self.threshold:
            print("human lost")
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
