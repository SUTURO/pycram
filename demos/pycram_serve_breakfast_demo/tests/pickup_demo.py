from demos.pycram_serve_breakfast_demo.utils.misc import *
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.world_concepts.world_object import Object

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning


# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_door_open_9.urdf")

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

robot.set_color([0.5, 0.5, 0.9, 1])


# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    TalkingMotion("Starting demo").perform()
    rospy.loginfo("Starting demo")
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    NavigateAction(target_locations=[Pose([2.24, 1.9, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    # MoveTorsoAction([0.2]).resolve().perform()
    LookAtAction(targets=[Pose([5.1, 2.1, 0.21], [0, 0, 0, 1])]).resolve().perform()
    object_desig = DetectAction(technique='all').resolve().perform()
    sort_objects = sort_objects(object_desig, wished_sorted_obj_list=["Metalbowl", "Cerealbox", "Milkpackja", "Spoon"])
    try_pick_up(robot, sort_objects[0], "top")

    # Move away from the table
    NavigateAction([Pose([robot.get_pose().pose.position.x - 0.15, robot.get_pose().pose.position.y,
                          0])]).resolve().perform()
