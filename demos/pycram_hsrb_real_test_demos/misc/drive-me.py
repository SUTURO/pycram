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


if __name__ == '__main__':
    navPose = Pose([2.7,1.9,0])
    move = navi.PoseNavigator()
    move.pub_now(navPose)


