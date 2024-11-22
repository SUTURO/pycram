import rospy
from tmc_msgs.msg import TalkRequestActionGoal


class TextToSpeechPublisher():

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('talk_request_action_goal_publisher', anonymous=True)

        # Create a publisher for the /talk_request_action/goal topic
        pub = rospy.Publisher('/talk_request_action/goal', TalkRequestActionGoal, queue_size=10)
        if self.pub.wait_for_server():
            rospy.loginfo("Waiting for talk ActionServer")
        else:
            rospy.loginfo("done waiting for talk action server")
        # Set the publishing rate (e.g., 1 Hz)
        rate = rospy.Rate(1)

        # Create an instance of the TalkRequestActionGoal message
        goal_msg = TalkRequestActionGoal()

        # Fill in the fields of the message
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal.data.language = 1
        goal_msg.goal.data.sentence = "hi"
        # Add other fields as required by your application

    def pub_now(self, sentence):
        # Log the message being published
        rospy.loginfo(sentence)
        self.goal_msg.goal.data.sentence = sentence
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)  # Sleep for 100ms and check again
        self.pub.publish(self.msg)

