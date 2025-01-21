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
    MoveTorsoAction([0.4]).resolve().perform()

    pickup = "popcorn_table"
    placing = "none"

    if pickup == "shelf":
        # shelf pickup
        #################################################################
        NavigateAction(target_locations=[Pose([4.5, 3.95, 0], [0, 0, 0, 1])]).resolve().perform()
        LookAtAction(targets=[Pose([5.3, 3.9, 0.21], [0, 0, 0, 1])]).resolve().perform()
        ##################################################################
    elif pickup == "long_table":
        # long table pickup
        #################################################################
        NavigateAction(target_locations=[Pose([4.45, 4.9, 0], [0, 0, 1, 0])]).resolve().perform()
        LookAtAction(targets=[Pose([3.5, 4.9, 0.3], [0, 0, 1, 0])]).resolve().perform()
        #################################################################
    elif pickup == "popcorn_table":
        # popcorn table pickup
        #################################################################
        NavigateAction(target_locations=[Pose([1.4, 3.9, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
        LookAtAction(targets=[Pose([1.5, 4.9, 0.4], [0, 0, 0.7, 0.7])]).resolve().perform()
        #################################################################

    object_desig = DetectAction(technique='all').resolve().perform()
    print(object_desig)
    obj_list = sort_objects(object_desig,  wished_sorted_obj_list=["Cerealbox"])
    PickUpAction(obj_list[0], [Arms.LEFT], [Grasp.FRONT]).resolve().perform()

    if pickup == "shelf":
        # shelf pickup
        #################################################################
        NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x - 0.6,
                                               robot.get_pose().pose.position.y, 0],
                                              [0, 0, 0, 1])]).resolve().perform()
        ##################################################################
    elif pickup == "long_table":
        # long table pickup
        #################################################################
        NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x + 0.6,
                                               robot.get_pose().pose.position.y, 0],
                                              [0, 0, 1, 0])]).resolve().perform()
        #################################################################
    elif pickup == "popcorn_table":
        # popcorn table pickup
        #################################################################
        NavigateAction(target_locations=[Pose([robot.get_pose().pose.position.x,
                                               robot.get_pose().pose.position.y - 0.6, 0],
                                              [0, 0, 0.7, 0.7])]).resolve().perform()
        #################################################################

    ParkArmsAction([Arms.LEFT]).resolve().perform()

    if placing == "popcorn_table":
        # popcorn table placing
        #################################################################
        # NavigateAction(target_locations=[Pose([3.9, 1.9, 0], [0, 0, 1, 0])]).resolve().perform()
        # NavigateAction(target_locations=[Pose([1.8, 1.9, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
        NavigateAction(target_locations=[Pose([1.4, 3.9, 0], [0, 0, 0.7, 0.7])]).resolve().perform()

        # PlaceGivenObjectAction(object_types=["Spoon"], arms=[Arms.LEFT],
        #                        target_locations=[Pose([1.4, 4.8, 0.775], [0, 0, 0.7, 0.7])],
        #                        grasps=[Grasp.TOP], on_table=True).resolve().perform()
        #TODO: change z + grasp
        PlaceAction(object_designator_description=obj_list[0], target_locations=[Pose([1.4, 4.8, 0.775], [0, 0, 0.7, 0.7])],
                    grasps=[Grasp.TOP], arms=[Arms.LEFT]).resolve().perform()
        NavigateAction(target_locations=[Pose([1.4, 3.9, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        # MoveTorsoAction([0]).resolve().perform()
        # LookAtAction(targets=[Pose([1.55, 4.9, 0.4], [0, 0, 0.7, 0.7])]).resolve().perform()
        # bowl_desig = DetectAction(technique='all').resolve().perform()
        # print(bowl_desig)
        # print(type(bowl_desig))
        # bowl = get_bowl(bowl_desig)
        # PouringAction([bowl.pose], [Arms.LEFT], ["right"], [115]).resolve().perform()
        ################################################################
    elif placing == "long_table":
        # long table placing
        ##################################################################
        NavigateAction(target_locations=[Pose([4.5, 4.9, 0], [0, 0, 1, 0])]).resolve().perform()
        PlaceAction(object_designator_description=obj_list[0], target_locations=[Pose([3.6, 4.9, 0.818])],
                    grasps=[Grasp.TOP], arms=[Arms.LEFT]).resolve().perform()
        #################################################################


    ###################################### OPEN DISHWASHER ##################################
    # NavigateAction(target_locations=[Pose([2.65, -2.25, 0], [0, 0, -1, 1])]).resolve().perform()
    # ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
    # Opendishwasher
    ########################################################################################



    ####################################### Demo Start #####################################
    # navigate from door to dishwasher
    # open dishwasher
    # navigate to popcorn table and perceive twice once from left one from right
    # merge found objects into one list
    # pick up objects that doesnt belong to the "serve breakfast" and place then in the dishwasher
    # after cleaning up the table look which objects still remaining and navigate to shelf to pick them up
    # pick up remaaining objects and place them on popcorn table with pouring

    # navigate_to(1.63, y, "popcorn table")
    # navigate_to(2.45, y, "popcorn table")


    # try_pick_up(robot, obj_list[0], "front")
    # Move back
    # NavigateAction(target_locations=[Pose([1.45, 5, 0], [0, 0, 0.7, 0.7])]).resolve().perform()
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    # PlaceAction(obj_list[0], ["left"], ["front"],
                # [Pose([3.4, 5.8, 0.78])]).resolve().perform()