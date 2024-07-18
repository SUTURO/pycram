from pycram.designators.action_designator import fts, ParkArmsAction, DetectAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.local_transformer import LocalTransformer
from pycram.plan_failures import SensorMonitoringCondition
from pycram.pose import Pose
from pycram.process_module import real_robot
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
import pycram.external_interfaces.giskard_new as giskardpy
import pycram.external_interfaces.robokudo as robokudo
import pycram.external_interfaces.navigate as navi
import rospy
import tf
from geometry_msgs.msg import PoseStamped

from pycram.ros.viz_marker_publisher import ManualMarkerPublisher

# # Initialize the necessary components
# world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()
# giskardpy.init_giskard_interface()
# robokudo.init_robokudo_interface()
# marker = ManualMarkerPublisher()
# listener = tf.TransformListener()


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

start_pose = Pose([1.5, 0.5, 0])
convo_pose = Pose([4.8, 0.49, 0],[0, 0, 1, 0])
convo_pose_to_couch = Pose([4.8, 0.49, 0],[0, 0, 0, 1])
couch_pose = Pose([8.6, 0, 0],[0, 0, 0, 1])
couch_pose_to_door = Pose([8.6, 0, 0],[0, 0, 1, 0])
diswasher = Pose([9, 3, 0],[0, 0, 0.7, 0.7])  # ori
schreibtisch = Pose([8.6, 0, 0],[0, 0, 1, 0])
schreibtisch_to_eingang = Pose([4.8, 1.1, 0],[0, 0, -0.7, 0.7])
if __name__ == '__main__':
    move = navi.PoseNavigator()
    rospy.sleep(1)
    rospy.loginfo("navi")
    move.pub_now(convo_pose_to_couch)
    move.pub_now(convo_pose)
    move.pub_now(convo_pose_to_couch)

    move.pub_now(couch_pose)
    move.pub_now(couch_pose_to_door)
    move.pub_now(couch_pose)

    move.pub_now(diswasher)
    move.pub_now(schreibtisch)
    move.pub_now(schreibtisch_to_eingang)
