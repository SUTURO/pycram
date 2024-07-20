from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
move = PoseNavigator()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the couch table
table_pose = 4.84

# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"

# Intermediate positions for a safer navigation
move_to_the_middle_table_pose = [2.2, 1.98, 0]
move_to_the_middle_dishwasher_pose = [2.2, -0.1, 0]

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_clean_v8.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")

dishwasher_main_name = "sink_area_dish_washer_main"


def get_placing_pos(obj):
    lt = LocalTransformer()
    dishwasher_main_name = "sink_area_dish_washer_main"
    link = apartment.get_link_tf_frame(dishwasher_main_name)

    world.current_bullet_world.add_vis_axis(apartment.get_link_pose(dishwasher_main_name))
    if obj == "Cutlery":
        # z = 0.48
        dishwasher = Pose([0.7225437770469778, 0.035078306407233306, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Metalbowl":
        dishwasher = Pose([0.5081242147912377, 0.15559490825966282, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Metalmug":
        dishwasher = Pose([0.5057047928413176, 0.10565347859021657, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Metalplate":
        # z = 0.55
        dishwasher = Pose([0.6229715158422948, -0.16033245276069596, 0.11000000238418584],
                          [0, 0, 0.9997071054536235, 0.024201307930308])
    elif obj == "Dishwashertab":
        # todo: Werte ändern
        dishwasher = Pose([0.5988132707204343, -0.03902136838407344, 0.04000000238418577],
                          [0, 0, 0.9997071054536235, 0.024201307930308])

    dishwasher.header.frame_id = link  # auskommentieren, wenn 1) verwendet
    newp = lt.transform_pose(dishwasher, "map")  # link statt map wenn 1) verwendet. map wenn 2) verwendet
    print(newp)
    world.current_bullet_world.add_vis_axis(newp)
    res = Pose([newp.pose.position.x, newp.pose.position.y, newp.pose.position.z],
               [newp.pose.orientation.x, newp.pose.orientation.y, newp.pose.orientation.z, newp.pose.orientation.w])
    return res


def calculate_placing_pos(x_pos, y_pos, z_pos):
    lt = LocalTransformer()

    link = apartment.get_link_tf_frame(dishwasher_main_name)

    world.current_bullet_world.add_vis_axis(apartment.get_link_pose(dishwasher_main_name))
    dishwasher = Pose([x_pos, y_pos, z_pos], [0, 0, 0, 1])
    newp = lt.transform_pose(dishwasher, link)  # link statt map wenn 1) verwendet. map wenn 2) verwendet
    print(newp)
    world.current_bullet_world.add_vis_axis(newp)
    return newp.pose


with real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    calculate_placing_pos(9.78,4.66, 0.48) # cutlery
    calculate_placing_pos(10, 4.64, 0.48) # metalmug
    calculate_placing_pos(10.15, 4.64, 0.48) # metalmug or bowl
    # calculate_placing_pos(9.87, 4.9, 0.55) # plate
    # calculate_placing_pos(9.9, 4.78, 0.48) # dishwashertab
    rospy.loginfo("Starting demo")

    # obj = "Metalmug"
    # pos = get_placing_pos(obj)
    # print(f"Metalmug: {pos}")
    #
    # pos = get_placing_pos("Metalplate")
    # print(f"Metalplate: {pos}")
    #
    # pos = get_placing_pos("Metalbowl")
    # print(f"Metalbowl: {pos}")
    # if obj in ["Metalbowl", "Metalmug"]:
    #     print("right")
    # else:
    #     print("front")
    # pos = get_placing_pos("Cutlery")
    # print(f"Cutlery: {pos}")
    # if obj in ["Metalbowl", "Metalmug"]:
    #     print("right")
    # else:
    #     print("front")

    # text_to_speech_publisher.pub_now("living room")
    # goal_pose = Pose([5.8, 0.2, 0], [0, 0, 0, 1])
    # move.pub_now(goal_pose)
    # time.sleep(1)
    #
    # text_to_speech_publisher.pub_now("middle living room")
    # goal_pose = Pose([5.8, 0.2, 0], [0, 0, 0, 1])
    # move.pub_now(goal_pose)
    # time.sleep(1)
    #
    # text_to_speech_publisher.pub_now("kitchen")
    # goal_pose = Pose([9.2, 3.08, 0], [0, 0, 0.7, 0.7])
    # move.pub_now(goal_pose)
    # time.sleep(1)
    #
    # text_to_speech_publisher.pub_now("Dishwasher")
    # goal_pose = Pose([9.18, 4.72, 0], [0, 0, 0, 1])
    # move.pub_now(goal_pose)
    # time.sleep(4)
    #
    # text_to_speech_publisher.pub_now("Table")
    # goal_pose = Pose([8.8, 4.76, 0], [0, 0, 1, 0])
    # move.pub_now(goal_pose)
    # MoveTorsoAction([0.2]).resolve().perform()
    # LookAtAction(targets=[Pose([7.8, 4.76, 0.25])]).resolve().perform()
    # time.sleep(2)
    #
    # text_to_speech_publisher.pub_now("Dishwasher")
    # goal_pose = Pose([9.18, 4.72, 0], [0, 0, 0, 1])
    # move.pub_now(goal_pose)
    # time.sleep(4)
    #
    # text_to_speech_publisher.pub_now("Dishwasher right")
    # goal_pose = Pose([9.8, 4.3, 0], [0, 0, 0.7, 0.7])
    # move.pub_now(goal_pose)
    # time.sleep(4)

    text_to_speech_publisher.pub_now("Done")
