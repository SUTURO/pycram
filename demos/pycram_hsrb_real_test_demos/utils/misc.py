import rospy
from docutils.nodes import math

from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from std_msgs.msg import String
from deprecated import deprecated
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.language import Code
from pycram.plan_failures import EnvironmentUnreachable, GripperClosedCompletely


def pickerino(object_desig, grasp, arm, talk):
    lt = LocalTransformer()
    robot = BulletWorld.robot
    # Retrieve object and robot from designators
    object = object_desig.bullet_world_object
    # Calculate the object's pose in the map frame
    oTm = object.get_pose()
    execute = True

    if grasp == "top":
        oTm.pose.position.z += 0.035

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
    oTb.orientation = grasp_rotation
    oTmG = lt.transform_pose(oTb, "map")

    rospy.logwarn("Opening Gripper")
    MoveGripperMotion(motion="open", gripper=arm).resolve().perform()

    rospy.logwarn("Picking up now")
    BulletWorld.current_bullet_world.add_vis_axis(oTmG)

    tool_frame = robot_description.get_tool_frame(arm)
    special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
    y = 0.04
    special_knowledge_offset.pose.position.y -= y
    if grasp == "top":
        z = 0.025
        special_knowledge_offset.pose.position.z -= z

    push_baseTm = lt.transform_pose(special_knowledge_offset, "map")
    special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")
    liftingTm = push_baseTm
    liftingTm.pose.position.z += 0.03
    talk.pub_now("Pick Up now!" + object.type)
    giskardpy.achieve_sequence_pick_up(oTmG, special_knowledge_offsetTm, push_baseTm, liftingTm)
    # MoveGripperMotion(motion="open", gripper=arm, allow_gripper_collision=True).resolve().perform()

    tool_frame = robot_description.get_tool_frame(arm)
    robot.attach(object=object_desig.bullet_world_object, link=tool_frame)


def pakerino():
    config = robot_description.get_static_joint_chain("left", "park")
    giskardpy.allow_all_collisions()
    giskardpy.achieve_joint_goal(config)
    MoveTorsoAction([0.1]).resolve().perform()

def monitor_func_place():
    der = fts.get_last_value()
    if der.wrench.force.x > 5.30:
        return SensorMonitoringCondition
    return False



def placerino(obj, grasp, arm, talk, target_location, kitchen_name):
    lt = LocalTransformer()
    robot = BulletWorld.robot
    oTm = target_location

    # if grasp == "top":
    #     oTm.pose.position.z += 0.035

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
    oTb.orientation = grasp_rotation
    special_knowledge_offset = oTb
    # x = 0.04
    # special_knowledge_offset.pose.position.x -= x
    if grasp == "top":
        y = 0.025
        special_knowledge_offset.pose.position.y -= y
    oTmG = lt.transform_pose(oTb, "map")
    push_baseTm = lt.transform_pose(special_knowledge_offset, "map")

    #tool_frame = robot_description.get_tool_frame(arm)
    #special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))


    #push_baseTm = lt.transform_pose(special_knowledge_offset, "map")
    #special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")
    # push_baseTm.pose.position.z -= 2
    talk.pub_now("Placing now!")
    #giskardpy.achieve_placing_without_prepose(oTmG, obj.name, kitchen_nameecho"Pane4";execbash)

    talk.pub_now("opening my gripper")
    MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    robot.detach_all()


