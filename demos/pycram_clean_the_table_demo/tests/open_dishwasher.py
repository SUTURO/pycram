
from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import StartSignalWaiter
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher


# Create an instance of the StartSignalWaiter


# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Fork", "Spoon", "Metalmug"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# ForceTorqueSensor for recognizing push on the hand
fts = ForceTorqueSensor(robot_name='hsrb')

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_clean_v1.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()

move = PoseNavigator()
move_to_living_room = Pose([5.8, 0.2, 0], [0, 0, 0, 1])
move_in_the_middle_of_living_room = Pose([8.4, 0.86, 0], [0, 0, 0, 1])
move_to_kitchen = Pose([9.2, 3.08, 0], [0, 0, 0.7, 0.7])
dishwasher_location_name = "dishwasher"
placing_location_name_left = "dishwasher_left"
placing_location_name_right = "dishwasher_right"
dishwasher_placing_pos = "dishwasher_front"
pickup_location_name = "dinner_table"
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"

# Intermediate positions for a safer navigation
move_to_the_middle_dishwasher_pose = Pose([8.9, 4.72, 0], [0, 0, 0, 1])
goal_pose = None

def navigate_to(location_name: str, y: Optional[float] = None):
    """
    Navigates to the couch table or to the dishwasher on different sides.

    :param y: y pose to navigate to the couch table for picking up objects
    :param location_name: defines the name of the location to move to
    """
    global goal_pose
    if location_name == dishwasher_placing_pos:
        goal_pose = Pose([9.18, 4.72, 0], [0, 0, 0, 1])
        move.pub_now(move_to_the_middle_dishwasher_pose)
        while not check_position():
            move.pub_now(goal_pose)
    else:
        rospy.logerr(f"Failure. Y-Value must be set for the navigateAction to the {pickup_location_name}")

def check_position():
    global goal_pose
    current_pose = robot.get_pose().pose.position
    euclidean_dist = math.sqrt(pow((goal_pose.pose.position.x - current_pose.x), 2) +
                               pow((goal_pose.pose.position.y - current_pose.y), 2))
    if euclidean_dist < 0.1:
        print("return true")
        return True
    print("return false")
    return False


with real_robot:
    rospy.loginfo("Starting demo")
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    # navigate to dishwasher
    move.pub_now(move_to_living_room)
    move.pub_now(move_in_the_middle_of_living_room)
    move.pub_now(move_to_kitchen)

    MoveJointsMotion(["wrist_roll_joint"], [-1.5]).resolve().perform()
    navigate_to(dishwasher_placing_pos)

    OpenDishwasherAction(handle_name, door_name, 0.6, 1.4, ["left"]).resolve().perform()

    text_to_speech_publisher.pub_now("Please pull out the lower rack")

    ParkArmsAction([Arms.LEFT]).resolve().perform()