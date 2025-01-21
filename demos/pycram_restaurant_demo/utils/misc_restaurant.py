import math
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped

from pycram.failures import SensorMonitoringCondition
from pycram.fluent import Fluent


class Restaurant:
    def __init__(self, robot: Any ,rospy):
        self.toya_pose = Fluent()
        self.human_pose = None
        self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)
        self.robot = robot
        self.rospy = rospy

    def toya_pose_cb(self, msg):
        #print("updating")
        self.toya_pose.set_value(self.robot.get_pose())
        self.rospy.sleep(0.5)

    def set_human_pose(self, pose: PoseWithCovarianceStamped):
        self.human_pose = pose

    def distance(self):
        print("toya pose:" + str(self.toya_pose.get_value().pose))
        if self.human_pose:
            dis = math.sqrt((self.human_pose.pose.position.x - self.toya_pose.get_value().pose.position.x) ** 2 +
                            (self.human_pose.pose.position.y - self.toya_pose.get_value().pose.position.y) ** 2)
            print("dis: " + str(dis))
            return dis
        else:
            self.rospy.logerr("Cant calculate distance, no human pose found")


def monitor_func(restaurant : Restaurant):
    if restaurant.distance() < 1:
        return SensorMonitoringCondition
    return False