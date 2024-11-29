import rospy

from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl, sort_objects, try_pick_up, get_free_spaces
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

#robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)


# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    # TalkingMotion("Starting demo").perform()
    # rospy.loginfo("Starting demo")

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    # NavigateAction(target_locations=[Pose([1.7, 2.4, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    NavigateAction(target_locations=[Pose([4.25, 3.95, 0], [0, 0, 0, 1])]).resolve().perform()
    LookAtAction(targets=[Pose([5.3, 3.9, 0.21], [0, 0, 0, 1])]).resolve().perform()
    # place_poses_list = DetectAction(technique='location', state='popcorn_table').resolve().perform()
    # sorted_places = get_free_spaces(place_poses_list[1])
    # print(sorted_places)
    object_desig = DetectAction(technique='all').resolve().perform()
    obj_list = sort_objects(object_desig,  wished_sorted_obj_list=["Cerealbox"])
    PickUpAction(obj_list[0], [Arms.LEFT], [Grasp.FRONT]).resolve().perform()
    NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x - 0.6,
                                           robot.get_pose().pose.position.y, 0],
                                          [0, 0, 0, 1])]).resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    NavigateAction(target_locations=[Pose([3.9, 1.9, 0], [0, 0, 1, 0])]).resolve().perform()
    NavigateAction(target_locations=[Pose([1.7, 2.4, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    NavigateAction(target_locations=[Pose([1.4, 4, 0], [0, 0, 0.7, 0.7])]).resolve().perform()

    PlaceAction(object_designator_description=obj_list[0], target_locations=[Pose([1.4, 4.8, 0.875], [0, 0, 1, 0])],
                grasps=[Grasp.FRONT], arms=[Arms.LEFT]).resolve().perform()


    # NavigateAction(target_locations=[Pose([4.5, 5, 0], [0, 0, 1, 0])]).resolve().perform()
    # PlaceAction(object_designator_description=obj_list[0], target_locations=[Pose([3.6, 5, 0.875], [0, 0, 1, 0])],
                # grasps=[Grasp.FRONT], arms=[Arms.LEFT]).resolve().perform()
    # try_pick_up(robot, obj_list[0], "front")
    # Move back
    # NavigateAction(target_locations=[Pose([1.45, 5, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    # PlaceAction(obj_list[0], ["left"], ["front"],
                # [Pose([3.4, 5.8, 0.78])]).resolve().perform()