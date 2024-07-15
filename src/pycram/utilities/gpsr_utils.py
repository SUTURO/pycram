import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code
from pycram.plan_failures import NoPlacePoseFoundCondition

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal, pakerino, GraspListener
from pycram.designator import LocationDesignatorDescription
import random

world = BulletWorld()
environment_raw = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_sg.urdf")
environment_desig = ObjectDesignatorDescription(names=["kitchen"])
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])

# mucho importante
grasp_listener = GraspListener()
talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()
lt = LocalTransformer()
gripper = HSRBMoveGripperReal()
previous_value = None



# return a pose if pose found otherwise NoPlacePoseFoundCondition
def get_place_poses_for_surface(link, object_to_place):
    place_poses = find_placeable_pose(link, environment_desig.resolve(), robot_desig.resolve(), "left",
                                      world, 0.1,
                                      object_desig=object_to_place)
    print(place_poses)
    if place_poses:
        nearest_pose = place_poses[0]

        pose_in_shelf = lt.transform_pose(nearest_pose, environment_raw.get_link_tf_frame(link))

        pose_in_shelf.pose.position.x = -0.10
        pose_in_shelf.pose.position.z += 0.03
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")

        world.current_bullet_world.add_vis_axis(adjusted_pose_in_map)

        return adjusted_pose_in_map

    else:
        return NoPlacePoseFoundCondition


def test_get_pose():
    # 'shelf:shelf:shelf_floor_0',
    links_from_shelf = ['shelf:shelf:shelf_floor_1', 'shelf:shelf:shelf_floor_2', ]
    popcorn_frame = "popcorn_table:p_table:table_front_edge_center"
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]),
                    color=[0, 1, 0, 1])
    spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
    bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])
    get_place_poses_for_surface(links_from_shelf[1], milk)


def place(object_type, object, grasp, target_location, link):
    def monitor_func_place():
        global previous_value
        der = fts.get_last_value()
        current_value = fts.get_last_value()

        prev_force_x = previous_value.wrench.force.x
        curr_force_x = current_value.wrench.force.x
        change_in_force_x = abs(curr_force_x - prev_force_x)
        print(f"Current Force X: {curr_force_x}, Previous Force X: {prev_force_x}, Change: {change_in_force_x}")

        def calculate_dynamic_threshold(previous_force_x):
            # Placeholder for a dynamic threshold calculation based on previous values
            # This function can be enhanced to calculate a threshold based on the history of values or other logic
            return max(0.1 * abs(previous_force_x), 1.5)  # Example: 10% of the previous value or a minimum of 1.5

        if change_in_force_x >= calculate_dynamic_threshold(previous_force_x=prev_force_x):
            print("Significant change detected")

            return SensorMonitoringCondition

        return False

    global previous_value
    oTm = target_location
    oTm.pose.position.z += 0.02

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, environment_raw.get_link_tf_frame(link))

    if grasp == "top":
        grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
        oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
    else:
        oTb.orientation = grasp_rotation

    oTmG = lt.transform_pose(oTb, "map")

    BulletWorld.current_bullet_world.add_vis_axis(oTmG)

    if grasp == "front":
        config_for_placing = {'arm_lift_joint': 0.20, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                              'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
    else:
        config_for_placing = {'arm_flex_joint': 0.20, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                              'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

    pakerino(config=config_for_placing)
    talk.pub_now(f"Placing now! {object_name.split('_')[0]} from: {grasp}")
    giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
    while not giskard_return:
        rospy.sleep(0.1)
    config_after_place = {'arm_lift_joint': 0.0}
    talk.pub_now("Placing neatly")
    previous_value = fts.get_last_value()
    try:
        plan = Code(lambda: giskardpy.test(config_after_place)) >> Monitor(monitor_func_place)
        plan.perform()

    except Exception as e:
        print(f"Exception type: {type(e).__name__}")

    giskardpy.achieve_attached(object, tip_link="map")
    BulletWorld.robot.detach(object)
    gripper.pub_now("open")

    giskardpy.avoid_all_collisions()
    park = pakerino()
    while not park:
        print("waiting for park")
        rospy.sleep(0.1)
    return True


#return grasped_bool, grasp, found_object
def process_pick_up_objects(obj_type, link):
    look_pose = environment_raw.get_link_pose(link)
    # park
    perceive_conf = {'arm_lift_joint': 0.20, 'wrist_flex_joint': 1.8, 'arm_roll_joint': -1, }
    pakerino(config=perceive_conf)
    # look
    giskardpy.move_head_to_pose(look_pose)

    talk.pub_now("perceiving", True)
    # noteme detect on table
    try:
        table_obj = DetectAction(technique='all').resolve().perform()
        found_object = None
        first, *remaining = table_obj
        for dictionary in remaining:
            for value in dictionary.values():
                if value.type == obj_type:
                    found_object = value
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        talk.pub_now("I was not able to perceive any objects")
        return

    # noteme if groups were found
    if found_object:
        obj_pose = found_object.bullet_world_object.pose
        object_raw = found_object
        tf_link = environment_raw.get_link_tf_frame(link)
        oTb = lt.transform_pose(obj_pose, tf_link)

        grasp_set = None
        # if to far behind the front face were robots look on the table
        if oTb.pose.position.x >= 0.20:
            grasp_set = "top"

        object_name = found_object.bullet_world_object.name
        object = found_object.bullet_world_object

        angle = helper.quaternion_to_angle(
            (oTb.pose.orientation.x, oTb.pose.orientation.y, oTb.pose.orientation.z, oTb.pose.orientation.w))
        object_dim = object.get_object_dimensions()
        print(f"obj dim von {object_name} {object_dim}")

        if grasp_set:
            grasp = "top"
        else:
            if object_dim[2] < 0.055:
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            elif object_dim[2] < 0.065 or angle > 40 and (object_dim[0] > 0.075 and object_dim[1] > 0.075):
                rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "top"
            else:
                rospy.logwarn(f"{object_name} grasp is set to front, angle: {angle}")
                rospy.logwarn(f"{object_name} and height {object_dim[2]}")
                rospy.logwarn(f"{object_name} and width {object_dim[0]}")
                grasp = "front"

            if grasp == "top":
                print("pose adjusted with z")
                oTb.pose.position.z += (object_dim[2] / 10)
                if object_dim[2] < 0.02:
                    rospy.logwarn(f"I am not able to grasp the object: {object_name} please help me!")
                    oTb.pose.position.z = 0.011
            else:
                oTb.pose.position.x += 0.03

        grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
        if grasp == "top":
            grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
            oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
        else:
            oTb.orientation = grasp_rotation

        oTmG = lt.transform_pose(oTb, "map")
        after_pose = oTmG.copy()
        after_pose.pose.position.z += 0.02

        BulletWorld.current_bullet_world.add_vis_axis(oTmG)

        if grasp == "front":
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
        else:
            config_for_placing = {'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                                  'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

        pakerino(config=config_for_placing)

        gripper.pub_now("open")
        talk.pub_now(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}")
        giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
        while not giskard_return:
            rospy.sleep(0.1)

        giskardpy.achieve_attached(object)
        tip_link = 'hand_gripper_tool_frame'
        BulletWorld.robot.attach(object=object, link=tip_link)
        gripper.pub_now("close")
        giskardpy.avoid_all_collisions()
        park = pakerino()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        grasped_bool = None
        if grasp_listener.check_grasp():
            talk.pub_now("Grasped a object")
            grasped_bool = True
        else:
            talk.pub_now("I was not able to grasped a object")
            grasped_bool = False

        return grasped_bool, grasp, found_object

#
# def follow_human():







# def demo(step):
#     global groups_in_shelf
#     with ((((real_robot)))):
#         object_name = None,
#         object = None,
#         grasp = None,
#         talk_bool = None,
#         target_location = None,
#         link = None
#         talk_bool = True
#         gripper.pub_now("close")
#         pakerino()
#
#         if step <= 1:
#             talk.pub_now("driving", True)
#             move.pub_now(rotated_shelf_pose, interrupt_bool=False)
#             move.pub_now(shelf_pose, interrupt_bool=False)
#             groups_in_shelf = process_objects_in_shelf(talk_bool)
#             move.pub_now(rotated_shelf_pose, interrupt_bool=False)
#
#         if step <= 2:
#             try:
#                 grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table = process_pick_up_objects(
#                     talk_bool)
#                 print(grasped_bool, grasp, group, object, obj_id, object_raw, groups_on_table)
#             except TypeError:
#                 print("done")
#                 return
#         if step <= 3:
#             move.pub_now(rotated_shelf_pose, interrupt_bool=False)
#             move.pub_now(shelf_pose, interrupt_bool=False)
#             place_pose, link = find_pose_in_shelf(group, object_raw, groups_in_shelf)
#             placeorpark(object.name, object, "front", talk_bool, place_pose, link, False)
#             demo(2)


# previous_value = fts.get_last_value()
#
# monitor_func_place()
# demo(0)
