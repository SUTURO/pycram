import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped
from robokudo_msgs.msg import QueryAction, QueryGoal

from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import fts, ParkArmsAction, DetectAction, NavigateAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.designators.object_designator import BelieveObject
from pycram.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.plan_failures import SensorMonitoringCondition
from pycram.pose import Pose
from pycram.process_module import real_robot
from utils.startup import startup
import pycram.external_interfaces.giskard_new as giskardpy

# Initialize the necessary components
world, v, talk, imgswap, move, robot = startup()
giskardpy.init_giskard_interface()
robokudo_client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
robokudo_client.wait_for_server()
rospy.loginfo("You can start your demo now")

# Initialize global variable
global human_bool
human_bool = False


def monitor_func():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def callback(point):
    global human_bool
    human_bool = True
    print("Human detected")


class Human:
    """
    Class that represents humans. this class does not spawn a human in a simulation.
    """

    def __init__(self):
        self.human_pose = Fluent()

        self.human_pose_sub = rospy.Subscriber("/cml_human_pose", PointStamped, self.human_pose_cb)

    def human_pose_cb(self, HumanPoseMsg):
        """
        callback function for human_pose Subscriber.
        sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic
        :param HumanPoseMsg: received message
        """

        self.human_pose.set_value(True)
        rospy.sleep(1)


# guiding -> human erkenn, human zu pose X bringen mit talken und allm
def guiding(talk_bool, poseTm):
    with real_robot:
        NavigateAction([poseTm]).resolve().perform()
        imgswap.pub_now(ImageEnum.HI.value)
        talk.pub_now("Hello, I am the robot. I am here to guide you to your destination. Let me search for you.",
                     talk_bool)
        imgswap.pub_now(ImageEnum.SEARCH.value)
        # human detect with fluent
        goal_msg = QueryGoal()
        robokudo_client.send_goal(goal_msg)
        human = Human()
        # wait for pose from human
        human.human_pose.wait_for()
        talk.pub_now("I have found you. Let me guide you to your destination. Please follow me.", talk_bool)
        # todo make a followME picture
        imgswap.pub_now(ImageEnum.FOLLOWING.value)
        # stopping perception
        robokudo_client.cancel_goal()
        NavigateAction([poseTm]).resolve().perform()

        while robot.pose.distance_to(poseTm) > 0.2:
            pass
        talk.pub_now("We have arrived at your destination. Thank you for following me.", talk_bool)
        imgswap.pub_now(ImageEnum.DONE.value)


def count_objects(type_of_object):
    # CAREFUL the type of object has be known from percepetion if you look forexample milk perception knows "Milkpack"
    obj_desig = ObjectDesignatorDescription(names=["object"], types=[type_of_object])

    with real_robot:
        # detecting all object with this type
        obj_dict = DetectAction(technique='all', object_designator=obj_desig).resolve().perform()
        # cut of the given State and keep the dictionary
        # Initialize a counter
        count = 0
        first, *remaining = obj_dict
        # calculate euclidian distance for all found object in a dictionary
        for dictionary in remaining:
            for value in dictionary.values():
                count += 1

        talk.pub_now("I have found " + str(count) + " " + type_of_object + "s.", True)
        return count


def gripper_action(state):
    with real_robot:
        MoveGripperMotion(state, "left").resolve().perform()


#example gripper action
#gripper_action("open")

# example guiding
# talk_bool = True
# navPoseguiding = Pose([1.6, 4, 0])
# guiding(talk_bool, navPoseguiding)


# example of counting
# count_objects returns the amount of objects of the given type
# count_objects("milk")


# # placing the object
#NavigateAction(target_locations=[Pose(move_to_the_middle_table_pose, [0, 0, 1, 0])]).resolve().perform()
# PlaceAction(sorted_obj[value], ["left"], [grasp],[Pose([x_pos, y_pos, z])]).resolve().perform()
# #ParkArmsAction([Arms.LEFT]).resolve().perform()
# PickUpAction(object, ["left"], [grasps]).resolve().perform()






# arranging

# transporting (also from human to human)

# looking for

# searching

# greeting

# take oder