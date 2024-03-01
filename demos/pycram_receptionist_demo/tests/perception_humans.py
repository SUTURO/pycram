import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.fluent import Fluent
from pycram.helper import axis_angle_to_quaternion
from pycram.process_module import semi_real_robot, real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String
from demos.pycram_receptionist_demo.deprecated import talk_actions
import pycram.external_interfaces.navigate as moveBase

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# carefull that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "kitchen.urdf")
giskardpy.init_giskard_interface()



def test():
    with real_robot:

        DetectAction(technique='human', state='start').resolve().perform()

        rospy.loginfo("human detected")

        print("---------- " + "start")

        rospy.loginfo("sleeping now")

        rospy.sleep(5)

        rospy.loginfo("sleep done stopping now")

        DetectAction(technique='human', state='stop').resolve().perform()

        print("------------------------------- stop")

        print("end")


if __name__ == '__main__':
    test()
