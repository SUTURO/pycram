import actionlib
import rospy
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import LaserScan, JointState
from sound_play.msg import SoundRequestActionGoal, SoundRequest
from std_msgs.msg import Int32
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestActionGoal
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *
from pycram.fluent import Fluent


def pakerino(torso_z=0.15, config=None):
    if not config:
        config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 1, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
                  'wrist_roll_joint': 0}
    return giskardpy.achieve_joint_goal(config)



class SoundRequestPublisher:
    """
    A class to publish sound requests in a ROS environment.
    """

    current_subscriber: rospy.Subscriber = None
    """
    Reference to the current subscriber instance.
    """

    def __init__(self, topic='/sound_play/goal', queue_size=10, latch=True):
        """
        Initializes the SoundRequestPublisher with a ROS publisher.

        :param topic: The ROS topic to publish sound requests to. Default is '/sound_play/goal'.
        :param queue_size: The size of the message queue for the publisher. Default is 10.
        :param latch: Whether the publisher should latch messages. Default is True.
        """
        self.pub = rospy.Publisher(topic, SoundRequestActionGoal, queue_size=queue_size, latch=latch)
        self.msg = SoundRequestActionGoal()
        self.msg.goal.sound_request.sound = 1
        self.msg.goal.sound_request.command = 1
        self.msg.goal.sound_request.volume = 2.0

    def publish_sound_request(self):
        """
        Publish the sound request message.
        """
        rospy.loginfo("Publishing sound request")
        rospy.loginfo("Waiting for subscribers to connect...")
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)  # Sleep for 100ms and check again
        self.pub.publish(self.msg)
        rospy.loginfo("Sound request published")


class StartSignalWaiter:
    """
    A class to wait for a start signal based on laser scan data in a ROS environment.
    """

    current_subscriber: rospy.Subscriber = None
    """
    Reference to the current subscriber instance.
    """

    def __init__(self):
        """
        Initializes the StartSignalWaiter with a Fluent object.
        """
        self.fluent = Fluent()
        self.current_subscriber = None

    def wait_for_startsignal(self):
        """
        Subscribe to the laser scan topic and wait for the door to open before continuing.
        """

        def laser_scan_callback(msg):
            ranges = list(msg.ranges)
            if len(ranges) > 481 and ranges[481] > 0.5:
                self.fluent.set_value(True)
                rospy.loginfo("Door is open, unsubscribing from topic")
                self.current_subscriber.unregister()

        rospy.loginfo("Waiting for starting signal.")
        self.current_subscriber = rospy.Subscriber("hsrb/base_scan", LaserScan, laser_scan_callback)

        rospy.loginfo("Waiting for door to open")
        self.fluent.wait_for()

        rospy.loginfo("Start signal received.")

    def something_in_the_way(self):
        """
        Subscribe to the laser scan topic and wait for an obstacle to appear within a specified distance
        before continuing.
        """

        def laser_scan_callback(msg):
            ranges = list(msg.ranges)
            rospy.loginfo(f"Received laser scan with {len(ranges)} ranges")
            # Ensure there are enough elements in ranges
            if len(ranges) > 501:
                obstacle_detected = False
                # Check if any value in the range 461 to 501 is smaller than 1.0
                for i in range(470, 510):
                    # print(ranges[i])
                    if ranges[i] < 0.98:
                        obstacle_detected = True
                        rospy.loginfo(f"Obstacle detected at index {i} with range {ranges[i]}")
                        break
                if obstacle_detected:
                    self.fluent.set_value(True)
                    rospy.loginfo("Obstacle detected within 1.0 meter, unsubscribing from topic")
                    self.current_subscriber.unregister()
                    return True
                else:
                    self.fluent.set_value(True)
                    rospy.loginfo("No Obstacle detected within 1.0 meter, unsubscribing from topic")
                    self.current_subscriber.unregister()
                    return False

        rospy.loginfo("Waiting for obstacle detection signal.")
        self.current_subscriber = rospy.Subscriber("hsrb/base_scan", LaserScan, laser_scan_callback)

        rospy.loginfo("Waiting for obstacle to be detected")
        self.fluent.wait_for()

        rospy.loginfo("Obstacle detection signal received.")





    def update_ros_parameters(new_params):
        """
        Update specified parameters on the ROS parameter server.

        :param new_params: Dictionary with new parameter values.
        """
        try:
            rospy.init_node('update_params_node', anonymous=True)

            for param, value in new_params.items():
                rospy.set_param(param, value)

            print("Parameters updated successfully.")

        except rospy.ROSInterruptException as e:
            print(f"An error occurred: {e}")



class TextToSpeechPublisher():

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

                    
class TextToImagePublisher:
    def __init__(self, topic='/head_display/text_to_image', queue_size=10):
        """
        Initializes the TextToImagePublisher with a ROS publisher.

        :param topic: The ROS topic to publish text to generate an image. Default is '/text_to_image'.
        :param queue_size: The size of the message queue for the publisher. Default is 10.
        """
        self.pub = rospy.Publisher(topic, String, queue_size=queue_size, latch=True)

    def pub_now(self, text):
        self.pub.publish(text)

        loginfo(f"Image generated for: {text}")


class ImageSwitchPublisher:
    """
    A class to publish image switch requests in a ROS environment.
    """

    def __init__(self, topic='/media_switch_topic', queue_size=10, latch=True):
        """
        Initializes the ImageSwitchPublisher with a ROS publisher.

        :param topic: The ROS topic to publish image switch requests to. Default is '/image_switch_topic'.
        :param queue_size: The size of the message queue for the publisher. Default is 10.
        :param latch: Whether the publisher should latch messages. Default is True.
        """
        self.pub = rospy.Publisher(topic, Int32, queue_size=queue_size, latch=latch)
        self.msg = Int32()

    def pub_now(self, image_id):
        """
        Publish the image switch request message.

        :param image_id: The ID of the image to switch to.
        """
        self.msg.data = image_id
        rospy.loginfo(f"Publishing image switch request with ID {image_id}")
        rospy.loginfo("Waiting for subscribers to connect...")
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)  # Sleep for 100ms and check again
        self.pub.publish(self.msg)
        rospy.loginfo("Image switch request published")


class HSRBMoveGripperReal():
    def __init__(self, latch=True):
        self.pub = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                   queue_size=10, latch=latch)
        self.msg = GripperApplyEffortActionGoal()

    def pub_now(self, motion: str):
        if motion == "open":
            self.msg.goal.effort = 0.8
            self.pub.publish(self.msg)
        elif motion == "close":
            self.msg.goal.effort = -0.8
            self.pub.publish(self.msg)


class GraspListener:
    def __init__(self):

        # Subscribe to the joint_states topic
        rospy.Subscriber("/hsrb/robot_state/joint_states", JointState, self.joint_states_callback)

        # Define the positions that indicate a grasp
        # Define the positions that indicate a grasp
        self.grasp_thresholds = {
            "hand_r_distal_joint": (-1.3, 0.76),  # Adjusted based on open and closed state data with offset
            "hand_l_distal_joint": (-1.3, 0.77)
        }
        # Open
        # State:
        # hand_r_distal_joint: -1.477408
        # hand_l_distal_joint: -1.477408
        # Closed
        # State:
        # hand_r_distal_joint: 0.800545
        # hand_l_distal_joint: 0.782546

        # Variable to store the grasp state
        self.grasped = False

    def check_grasp(self):
        print(self.grasped)
        return self.grasped

    def joint_states_callback(self, msg):
        # Extract the position of the relevant joints
        try:
            hand_r_index = msg.name.index("hand_r_distal_joint")
            hand_l_index = msg.name.index("hand_l_distal_joint")

            hand_r_position = msg.position[hand_r_index]
            hand_l_position = msg.position[hand_l_index]

            # Check if the positions are within the grasping thresholds
            if (self.grasp_thresholds["hand_r_distal_joint"][0] <= hand_r_position <=
                    self.grasp_thresholds["hand_r_distal_joint"][1] and
                    self.grasp_thresholds["hand_l_distal_joint"][0] <= hand_l_position <=
                    self.grasp_thresholds["hand_l_distal_joint"][1]):
                self.grasped = True
            else:
                self.grasped = False

            # rospy.loginfo("Grasp state: %s", "Grasped" if self.grasped else "Not grasped")

        except ValueError as e:
            rospy.logerr("Joint names not found in joint_states message: %s", e)

# Hints: List for image view (mit Zahlen ändert man das Bild)
# "hi.png" -> 0
# "talk.png" -> 1
# "dish.png" -> 2
# "done.png" -> 3
# "drop.png" -> 4
# "handover.png" -> 5
# "order.png" -> 6
# "picking.png" -> 7
# "placing.png" -> 8
# "repeat.png" -> 9
# "search.png" -> 10
# "waving.mp4" -> 11
# "following" -> 12


# Example usage:
# image_switch_publisher = ImageSwitchPublisher()
# image_switch_publisher.publish_image_switch(12)
# Publishing and latching message. Press ctrl-C to terminate.
