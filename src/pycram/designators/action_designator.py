import itertools
import time
from typing import Any, Union
from geometry_msgs.msg import WrenchStamped

from pycram.ros.force_torque_sensor import ForceTorqueSensor
import itertools
import math
import rospy
import sqlalchemy.orm
from typing import Any, Union
import rospy

from .location_designator import CostmapLocation
from .motion_designator import *
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from .. import helper
from ..bullet_world import BulletWorld
from ..designator import ActionDesignatorDescription
from ..enums import Arms, ObjectType
from ..helper import multiply_quaternions, axis_angle_to_quaternion, multiply_quaternions_by_vanessa
from ..language import Monitor
from ..local_transformer import LocalTransformer
from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     Action as ORMAction, LookAtAction as ORMLookAtAction,
                                     DetectAction as ORMDetectAction, TransportAction as ORMTransportAction,
                                     OpenAction as ORMOpenAction, CloseAction as ORMCloseAction,
                                     GraspingAction as ORMGraspingAction)

from ..orm.base import Quaternion, Position, Base
from ..plan_failures import ObjectUnfetchable, ReachabilityFailure, EnvironmentUnreachable, GripperClosedCompletely, \
    SensorMonitoringCondition
from ..pose import Pose
from ..robot_descriptions import robot_description
from ..ros.viz_marker_publisher import ManualMarkerPublisher
from ..task import with_tree


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Performable Move Torso Action designator.
        """

        position: float
        """
        Target position of the torso joint
        """

        @with_tree
        def perform(self) -> None:
            MoveJointsMotion([robot_description.torso_joint], [self.position]).resolve().perform()

        def to_sql(self) -> ORMMoveTorsoAction:
            return ORMMoveTorsoAction(self.position)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMMoveTorsoAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, positions: List[float], resolver=None):
        """
        Create a designator description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        :param resolver: An optional resolver that returns a performable designator for a designator description.
        """
        super().__init__(resolver)
        self.positions: List[float] = positions

    def ground(self) -> Action:
        """
        Creates a performable action designator with the first element from the list of possible torso heights.

        :return: A performable action designator
        """
        return self.Action(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator and returns a performable action designator with the value.

        :return: A performable action designator
        """
        for position in self.positions:
            yield self.Action(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        """
        The gripper that should be set 
        """
        motion: str
        """
        The motion that should be set on the gripper
        """

        @with_tree
        def perform(self) -> None:
            MoveGripperMotion(gripper=self.gripper, motion=self.motion).resolve().perform()

        def to_sql(self) -> ORMSetGripperAction:
            return ORMSetGripperAction(self.gripper, self.motion)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMSetGripperAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, grippers: List[str], motions: List[str], resolver=None):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        :param resolver: An alternative resolver that returns a performable designator for a designator description
        """
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.motions: List[str] = motions

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element in the grippers and motions list.

        :return: A performable designator
        """
        return self.Action(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield self.Action(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription, resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description = object_designator_description

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground())


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object
        effort: float

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 efforts: List[float], resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        arm: Arms
        """
        Entry from the enum for which arm should be parked
        """

        @with_tree
        def perform(self) -> None:
            # create the keyword arguments
            kwargs = dict()

            # add park left arm if wanted
            if self.arm in [Arms.LEFT, Arms.BOTH]:
                kwargs["left_arm_config"] = "park"
                MoveArmJointsMotion(**kwargs).resolve().perform()
                # MoveTorsoAction([0.005]).resolve().perform()
                MoveTorsoAction([0.10]).resolve().perform()
            # add park right arm if wanted
            if self.arm in [Arms.RIGHT, Arms.BOTH]:
                kwargs["right_arm_config"] = "park"
                MoveArmJointsMotion(**kwargs).resolve().perform()

        def to_sql(self) -> ORMParkArmsAction:
            return ORMParkArmsAction(self.arm.name)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMParkArmsAction:
            print("in insert parkArms")
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, arms: List[Arms], resolver=None):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        :param resolver: An optional resolver that returns a performable designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[Arms] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        return self.Action(self.arms[0])


class PickUpAction(ActionDesignatorDescription):
    """
    A class representing a designator for a pick-up action, allowing a robot to pick up a specified object.

    This class encapsulates the details of the pick-up action, including the object to be picked up, the arm to be used,
    and the grasp type. It defines the sequence of operations for the robot to execute the pick-up action, such as opening
    the gripper, moving to the object, closing the gripper, and lifting the object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be picked up
        """

        arm: str
        """
        The arm that should be used for pick up
        """

        grasp: str
        """
        The grasp that should be used. For example, 'top' or 'front'
        """

        @with_tree
        def perform(self) -> None:
            # Initialize the local transformer and robot reference
            lt = LocalTransformer()
            robot = BulletWorld.robot
            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object
            # Calculate the object's pose in the map frame
            oTm = object.get_pose()
            execute = True
            # todo workaround to get object frame
            # print("this is a test")
            # print(object.tf_frame)
            # cutlery_pose = lt.transform_pose(oTm, object.tf_frame)
            # while execute:
            #     print(object.get_object_dimensions())
            #     print("heheheheh")
            # # Adjust object pose for top-grasping, if applicable
            if self.grasp == "top":
                oTm.pose.position.z += 0.035
                # if object.type == "Cutlery":
                #     cutlery_pose = lt.transform_pose(oTm, object.tf_frame)
                #     cutlery_pose.pose.position.x -= 0.3

            pose_in_table = lt.transform_pose(oTm,
                                    BulletWorld.current_bullet_world.environment.get_link_tf_frame("dinner_table:dinner_table:table_center"))
            print(pose_in_table.pose.orientation)
            # Determine the grasp orientation and transform the pose to the base link frame
            grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
            # oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            # Set pose to the grasp rotation

            if self.grasp == "front" or self.object_designator.type == "Metalbowl":
                pose_in_table.pose.orientation.x = grasp_rotation[0]
                pose_in_table.pose.orientation.y = grasp_rotation[1]
                pose_in_table.pose.orientation.z = grasp_rotation[2]
                pose_in_table.pose.orientation.w = grasp_rotation[3]
            # todo removed rotation for bowl?
            elif self.grasp == "top" and self.object_designator.type != "Metalbowl":
                angle = helper.quaternion_to_angle(
                    (pose_in_table.pose.orientation.x, pose_in_table.pose.orientation.y, pose_in_table.pose.orientation.z,
                     pose_in_table.pose.orientation.w))
                print("angle: " + str(angle))
                grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
                if angle > 110:
                    print("greater: " + str(angle))
                    pose_in_table.pose.orientation.x = grasp_rotation[0]
                    pose_in_table.pose.orientation.y = grasp_rotation[1]
                    pose_in_table.pose.orientation.z = grasp_rotation[2]
                    pose_in_table.pose.orientation.w = grasp_rotation[3]

                else:
                    print("smaller: " + str(angle))
                    new_q = multiply_quaternions_by_vanessa(pose_in_table.pose.orientation, grasp_q)
                    pose_in_table.pose.orientation.x = new_q[0]
                    pose_in_table.pose.orientation.y = new_q[1]
                    pose_in_table.pose.orientation.z = new_q[2]
                    pose_in_table.pose.orientation.w = new_q[3]
            # Transform the pose to the map frame
            oTmG = lt.transform_pose(pose_in_table, "map")

            # Open the gripper before picking up the object
            rospy.logwarn("Opening Gripper")
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # Move to the pre-grasp position and visualize the action
            rospy.logwarn("Picking up now")
            BulletWorld.current_bullet_world.add_vis_axis(oTmG)
            # Execute Bool, because sometimes u only want to visualize the poses to pp.py things
            if execute:
                MoveTCPMotion(oTmG, self.arm, allow_gripper_collision=False).resolve().perform()

            # Calculate and apply any special knowledge offsets based on the robot and object type
            # Note: This currently includes robot-specific logic that should be generalized
            tool_frame = robot_description.get_tool_frame(self.arm)
            special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
            # hsrb specific
            if robot.name == "hsrb":
                if self.grasp == "top":
                    if self.object_designator.type == "Metalbowl":
                        special_knowledge_offset.pose.position.y += 0.085
                        special_knowledge_offset.pose.position.x -= 0.03

            push_base = special_knowledge_offset
            # hsrb specific
            if robot.name == "hsrb":
                z = 0.04
                if self.grasp == "top":
                    z = 0.025
                    if self.object_designator.type == "Metalbowl":
                        z = 0.044
                push_base.pose.position.z += z
            push_baseTm = lt.transform_pose(push_base, "map")
            special_knowledge_offsetTm = lt.transform_pose(push_base, "map")

            # Grasping from the top inherently requires calculating an offset, whereas front grasping involves
            # slightly pushing the object forward.
            rospy.logwarn("Offset now")
            # m = ManualMarkerPublisher()
            # m.create_marker("pose_pickup", special_knowledge_offsetTm)
            BulletWorld.current_bullet_world.add_vis_axis(special_knowledge_offsetTm)
            if execute:
                MoveTCPMotion(special_knowledge_offsetTm, self.arm, allow_gripper_collision=False).resolve().perform()

            rospy.logwarn("Pushing now")
            BulletWorld.current_bullet_world.add_vis_axis(push_baseTm)
            if execute:
                MoveTCPMotion(push_baseTm, self.arm, allow_gripper_collision=False).resolve().perform()

            # Finalize the pick-up by closing the gripper and lifting the object
            rospy.logwarn("Close Gripper")
            MoveGripperMotion(motion="close", gripper=self.arm, allow_gripper_collision=True).resolve().perform()

            rospy.logwarn("Lifting now")
            liftingTm = push_baseTm
            liftingTm.pose.position.z += 0.03
            BulletWorld.current_bullet_world.add_vis_axis(liftingTm)
            if execute:
                MoveTCPMotion(liftingTm, self.arm, allow_gripper_collision=False).resolve().perform()
            tool_frame = robot_description.get_tool_frame(self.arm)
            robot.attach(object=self.object_designator.bullet_world_object, link=tool_frame)

        def to_sql(self) -> ORMPickUpAction:
            return ORMPickUpAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            action = super().insert(session)
            # try to create the object designator
            if self.object_at_execution:
                od = self.object_at_execution.insert(session, )
                action.object = od.id
            else:
                action.object = None

            session.add(action)
            session.commit()

            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str], grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return self.Action(obj_desig, self.arms[0], self.grasps[0])


fts = ForceTorqueSensor(robot_name='hsrb')
pr = True


def monitor_func():
    der: WrenchStamped() = fts.get_last_value()
    print(abs(der.wrench.force.y))
    if abs(der.wrench.force.y) > 0.45:
        print(abs(der.wrench.force.y))
        print(abs(der.wrench.torque.y))
        return SensorMonitoringCondition
    return False


class PlaceAction(ActionDesignatorDescription):
    """
     A class representing a designator for a place action, allowing a robot to place a specified object.

    This class encapsulates the details of the place action, including the object to be placed, the arm to be used,
    the target_location to place the object and the grasp type. It defines the sequence of operations for the robot
    to execute the place action, such as moving the arm holding the object to the target_location, opening
    the gripper, and lifting the arm.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be placed
        """
        arm: str
        """
        Arm that is currently holding the object
        """

        grasp: str
        """
        Grasp that was used to pick up the object
        """
        target_location: Pose
        """
        Pose in the world at which the object should be placed
        """

        @with_tree
        def perform(self) -> None:
            lt = LocalTransformer()
            robot = BulletWorld.robot
            execute = True
            # oTm = Object Pose in Frame map
            oTm = self.target_location

            if self.grasp == "top":
                oTm.pose.position.z += 0.05

            # Determine the grasp orientation and transform the pose to the base link frame
            grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            # Set pose to the grasp rotation
            oTb.orientation = grasp_rotation
            # Transform the pose to the map frame
            oTmG = lt.transform_pose(oTb, "map")

            rospy.logwarn("Placing now")
            BulletWorld.current_bullet_world.add_vis_axis(oTmG)
            if execute:
                MoveTCPMotion(oTmG, self.arm).resolve().perform()

            tool_frame = robot_description.get_tool_frame(self.arm)
            push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
            if robot.name == "hsrb":
                z = 0.03
                if self.grasp == "top":
                    z = 0.07
                push_base.pose.position.z += z
            # todo: make this for other robots
            push_baseTm = lt.transform_pose(push_base, "map")

            rospy.logwarn("Pushing now")
            BulletWorld.current_bullet_world.add_vis_axis(push_baseTm)
            if execute:
                MoveTCPMotion(push_baseTm, self.arm).resolve().perform()
            if self.object_designator.type == "Metalplate":
                loweringTm = push_baseTm
                loweringTm.pose.position.z -= 0.08
                BulletWorld.current_bullet_world.add_vis_axis(loweringTm)
                if execute:
                    MoveTCPMotion(loweringTm, self.arm).resolve().perform()
                # rTb = Pose([0,-0.1,0], [0,0,0,1],"base_link")
                rospy.logwarn("sidepush monitoring")
                TalkingMotion("sidepush.").resolve().perform()
                side_push = Pose(
                    [push_baseTm.pose.position.x, push_baseTm.pose.position.y + 0.08, push_baseTm.pose.position.z],
                    [push_baseTm.orientation.x, push_baseTm.orientation.y, push_baseTm.orientation.z,
                     push_baseTm.orientation.w])
                try:
                    plan = MoveTCPMotion(side_push, self.arm) >> Monitor(monitor_func)
                    plan.perform()
                except (SensorMonitoringCondition):
                    rospy.logwarn("Open Gripper")
                    MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # Finalize the placing by opening the gripper and lifting the arm
            rospy.logwarn("Open Gripper")
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            rospy.logwarn("Lifting now")
            liftingTm = push_baseTm
            liftingTm.pose.position.z += 0.08
            BulletWorld.current_bullet_world.add_vis_axis(liftingTm)
            if execute:
                MoveTCPMotion(liftingTm, self.arm).resolve().perform()

        def to_sql(self) -> ORMPlaceAction:
            return ORMPlaceAction(self.arm)

        def insert(self, session, *args, **kwargs) -> ORMPlaceAction:
            action = super().insert(session)

            if self.object_designator:
                od = self.object_designator.insert(session, )
                action.object = od.id
            else:
                action.object = None

            if self.target_location:
                position = Position(*self.target_location.position_as_list())
                orientation = Quaternion(*self.target_location.orientation_as_list())
                session.add(position)
                session.add(orientation)
                session.commit()
                action.position = position.id
                action.orientation = orientation.id
            else:
                action.position = None
                action.orientation = None

            session.add(action)
            session.commit()
            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str], grasps: List[str], target_locations: List[Pose], resolver=None):
        """
        Lets the robot place an object. The description needs an object designator describing the object that should be
        placed, an arm that should be used as well as the grasp from which side the object was picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param target_locations: List of possible target locations for the object to be placed
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of
                         possible paramter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return self.Action(obj_desig, self.arms[0], self.grasps[0], self.target_locations[0])


class PlaceGivenObjAction(ActionDesignatorDescription):
    """
       A class representing a designator for a place action of human given objects, allowing a robot to place a
     human given object, that could not be picked up or were not found in the FOV.

    This class encapsulates the details of the place action of human given objects, including the type of the object to
    be placed, the arm to be used, the target_location to place the object and the grasp type. It defines the sequence
    of operations for the robot to execute the place action of human given object, such as moving the arm holding the
    object to the target_location, opening the gripper, and lifting the arm.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_type: str
        """
        Object type describing the object that should be placed
        """

        arm: str
        """
        Arm that is currently holding the object
        """

        target_location: Pose
        """
        Pose in the world at which the object should be placed
        """

        grasp: str
        """
        Grasp that defines how to place the given object
        """

        on_table: Optional[bool]
        """
        When placing a plate needed to differentiate between placing in a dishwasher and placing on the table. 
        Default is placing on a table.
        """

        @with_tree
        def perform(self) -> None:
            lt = LocalTransformer()
            robot = BulletWorld.robot
            # oTm = Object Pose in Frame map
            oTm = self.target_location
            execute = True

            # TODO add for other robots
            if self.object_type == "Metalplate" and self.on_table and robot.name == "hsrb":

                grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
                oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
                oTb.orientation = grasp_rotation
                oTmG = lt.transform_pose(oTb, "map")

                rospy.logwarn("Placing now")
                MoveTCPMotion(oTmG, self.arm).resolve().perform()

                MoveTorsoAction([0.62]).resolve().perform()
                kwargs = dict()

                # taking in the predefined arm configuration for placing
                if self.arm in ["left", "both"]:
                    kwargs["left_arm_config"] = "place_plate"
                    MoveArmJointsMotion(**kwargs).resolve().perform()

                # turning the gripper downwards to better drop the plate
                MoveJointsMotion(["wrist_flex_joint"], [-0.8]).resolve().perform()

                # correct a possible sloped orientation
                NavigateAction(
                    [Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y,
                             0])]).resolve().perform()

                MoveGripperMotion(motion="open", gripper="left").resolve().perform()

                # Move away from the table
                # todo generalize so that hsr is always moving backwards
                NavigateAction(
                    [Pose([robot.get_pose().pose.position.x - 0.1, robot.get_pose().pose.position.y,
                           0])]).resolve().perform()

            # placing everything else or the Metalplate in the dishwasher
            else:
                if self.grasp == "top":
                    oTm.pose.position.z += 0.05

                # Determine the grasp orientation and transform the pose to the base link frame
                grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
                oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
                # Set pose to the grasp rotation
                oTb.orientation = grasp_rotation
                # Transform the pose to the map frame
                oTmG = lt.transform_pose(oTb, "map")

                rospy.logwarn("Placing now")
                BulletWorld.current_bullet_world.add_vis_axis(oTmG)
                if execute:
                    MoveTCPMotion(oTmG, self.arm).resolve().perform()

                tool_frame = robot_description.get_tool_frame(self.arm)
                push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
                if robot.name == "hsrb":
                    z = 0.03
                    if self.grasp == "top":
                        z = 0.07
                    push_base.pose.position.z += z
                # todo: make this for other robots
                push_baseTm = lt.transform_pose(push_base, "map")

                rospy.logwarn("Pushing now")
                BulletWorld.current_bullet_world.add_vis_axis(push_baseTm)
                if execute:
                    MoveTCPMotion(push_baseTm, self.arm).resolve().perform()
                if self.object_type == "Metalplate":
                    loweringTm = push_baseTm
                    loweringTm.pose.position.z -= 0.08
                    BulletWorld.current_bullet_world.add_vis_axis(loweringTm)
                    if execute:
                        MoveTCPMotion(loweringTm, self.arm).resolve().perform()
                    # rTb = Pose([0,-0.1,0], [0,0,0,1],"base_link")
                    rospy.logwarn("sidepush monitoring")
                    TalkingMotion("sidepush.").resolve().perform()
                    side_push = Pose(
                        [push_baseTm.pose.position.x, push_baseTm.pose.position.y + 0.08, loweringTm.pose.position.z],
                        [push_baseTm.orientation.x, push_baseTm.orientation.y, push_baseTm.orientation.z,
                         push_baseTm.orientation.w])
                    try:
                        plan = MoveTCPMotion(side_push, self.arm) >> Monitor(monitor_func)
                        plan.perform()
                    except (SensorMonitoringCondition):
                        rospy.logwarn("Open Gripper")
                        MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

                # Finalize the placing by opening the gripper and lifting the arm
                rospy.logwarn("Open Gripper")
                MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

                rospy.logwarn("Lifting now")
                liftingTm = push_baseTm
                liftingTm.pose.position.z += 0.08
                BulletWorld.current_bullet_world.add_vis_axis(liftingTm)
                if execute:
                    MoveTCPMotion(liftingTm, self.arm).resolve().perform()

    def __init__(self,
                 object_types: List[str], arms: List[str], target_locations: List[Pose], grasps: List[str],
                 on_table: Optional[bool] = True, resolver=None):
        """
        Lets the robot place a human given object. The description needs an object type describing the object that
        should be placed, an arm that should be used as well as the target location where the object should be placed
        and the needed grasping movement.

        :param object_types: List of possible object types
        :param arms: List of possible arms that could be used
        :param target_locations: List of possible target locations for the object to be placed
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of
                         possible paramter
        """
        super().__init__(resolver)
        self.object_types: List[str] = object_types
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps
        self.target_locations: List[Pose] = target_locations
        self.on_table: bool = on_table

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible
        parameter.

        :return: A performable designator
        """
        return self.Action(self.object_types[0], self.arms[0], self.target_locations[0], self.grasps[0], self.on_table)


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target_location: Pose
        """
        Location to which the robot should be navigated
        """

        @with_tree
        def perform(self) -> None:
            MoveMotion(self.target_location).resolve().perform()

        def to_sql(self) -> ORMNavigateAction:
            return ORMNavigateAction()

        def insert(self, session, *args, **kwargs) -> ORMNavigateAction:
            action = super().insert(session)

            pose = self.target_location.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, target_locations: List[Pose], resolver=None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative resolver that creates a performable designator from the list of possible parameter
        """
        super().__init__(resolver)
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return self.Action(self.target_locations[0])


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be transported.
        """
        arm: str
        """
        Arm that should be used
        """
        target_location: Pose
        """
        Target Location to which the object should be transported
        """

        @with_tree
        def perform(self) -> None:
            robot_desig = BelieveObject(names=[robot_description.name])
            ParkArmsAction.Action(Arms.BOTH).perform()
            pickup_loc = CostmapLocation(target=self.object_designator, reachable_for=robot_desig.resolve(),
                                         reachable_arm=self.arm)
            # Tries to find a pick-up posotion for the robot that uses the given arm
            pickup_pose = None
            for pose in pickup_loc:
                if self.arm in pose.reachable_arms:
                    pickup_pose = pose
                    break
            if not pickup_pose:
                raise ObjectUnfetchable(
                    f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

            NavigateAction([pickup_pose.pose]).resolve().perform()
            PickUpAction.Action(self.object_designator, self.arm, "front").perform()
            ParkArmsAction.Action(Arms.BOTH).perform()
            try:
                place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig.resolve(),
                                            reachable_arm=self.arm).resolve()
            except StopIteration:
                raise ReachabilityFailure(
                    f"No location found from where the robot can reach the target location: {self.target_location}")
            NavigateAction([place_loc.pose]).resolve().perform()
            PlaceAction.Action(self.object_designator, self.arm, self.target_location).perform()
            ParkArmsAction.Action(Arms.BOTH).perform()

        def to_sql(self) -> ORMTransportAction:
            return ORMTransportAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMTransportAction:
            action = super().insert(session)

            od = self.object_designator.insert(session)
            action.object_id = od.id

            pose = self.target_location.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()

            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str], target_locations: List[Pose], resolver=None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator description or a specified Object designator that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()
        return self.Action(obj_desig, self.arms[0], self.target_locations[0])


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target: Pose
        """
        Position at which the robot should look, given as 6D pose
        """

        @with_tree
        def perform(self) -> None:
            LookingMotion(target=self.target).resolve().perform()

        def to_sql(self) -> ORMLookAtAction:
            return ORMLookAtAction()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMLookAtAction:
            action = super().insert(session)

            pose = self.target.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()
            return action

    def __init__(self, targets: List[Pose], resolver=None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative resolver that returns a performable designator for a list of possible target locations
        """
        super().__init__(resolver)
        self.targets: List[Pose] = targets

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return self.Action(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object with given technique.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        technique: str
        """
        Technique means how the object should be detected, e.g. 'color', 'shape', 'region', etc. 
        Or 'all' if all objects should be detected
        """

        object_designator: Optional[ObjectDesignatorDescription] = None
        """
        Object designator loosely describing the object, e.g. only type. 
        """

        state: Optional[str] = None
        """
        The state instructs our perception system to either start or stop the search for an object or human.
        Can also be used to describe the region or location where objects are perceived.
        """

        @with_tree
        def perform(self) -> Any:
            if self.object_designator:
                object_type = self.object_designator.types[0]
            else:
                object_type = None
            return DetectingMotion(technique=self.technique, object_type=object_type,
                                   state=self.state).resolve().perform()

        def to_sql(self) -> ORMDetectAction:
            return ORMDetectAction()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMDetectAction:
            action = super().insert(session)

            od = self.object_designator.insert(session)
            action.object_id = od.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, technique, resolver=None,
                 object_designator: Optional[ObjectDesignatorDescription] = None,
                 state: Optional[str] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param technique: Technique means how the object should be detected, e.g. 'color', 'shape', etc.
        :param object_designator_description: Object designator describing the object
        :param state: The state instructs our perception system to either start or stop the search for an object or human.
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.technique: str = technique
        self.object_designator: Optional[ObjectDesignatorDescription] = object_designator
        self.state: Optional[str] = state

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description.

        :return: A performable designator
        """

        return self.Action(technique=self.technique, object_designator=self.object_designator, state=self.state)


class OpenDishwasherAction(ActionDesignatorDescription):
    """
    Opens the dishwasher door
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        handle_name: str
        """
        Name of the handle to grasp for opening
        """

        door_name: str
        """
        Name of the door belonging to the handle
        """

        goal_state_half_open: float
        """
        goal state for opening the door partially
        """

        goal_state_full_open: float
        """
        goal state for opening the door fully
        """

        arm: str
        """
        Arm that should be used for opening the container
        """

        @with_tree
        def perform(self) -> Any:
            # Grasping the dishwasher handle
            MoveGripperMotion("open", self.arm).resolve().perform()
            GraspingDishwasherHandleMotion(self.handle_name, self.arm).resolve().perform()

            # partially opening the dishwasher door
            MoveGripperMotion("close", self.arm).resolve().perform()
            HalfOpeningDishwasherMotion(self.handle_name, self.goal_state_half_open, self.arm).resolve().perform()

            # moves arm around the door to further push it open
            MoveGripperMotion("open", self.arm).resolve().perform()
            MoveArmAroundMotion(self.handle_name, self.arm).resolve().perform()

            # pushes the rest of the door open
            MoveGripperMotion("close", self.arm).resolve().perform()
            FullOpeningDishwasherMotion(self.handle_name, self.door_name, self.goal_state_full_open,
                                        self.arm).resolve().perform()

            ParkArmsAction([self.arm]).resolve().perform()
            MoveGripperMotion("open", self.arm).resolve().perform()
            # plan = talk | park | gripper_open
            # plan.perform()

    def __init__(self, handle_name: str, door_name: str, goal_state_half_open: float, goal_state_full_open: float,
                 arms: List[str], resolver=None):
        """
        Moves the arm of the robot to open a container.

        :param handle_name: name of the dishwasher handle
        :param door_name: name of the belonging dishwasher door
        :param goal_state_half_open: state to open the dishwasher door partially
        :param goal_state_full_open: state to open the dishwasher door fully
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter.
        """
        super().__init__(resolver)
        self.handle_name = handle_name
        self.door_name = door_name
        self.goal_state_half_open = goal_state_half_open
        self.goal_state_full_open = goal_state_full_open
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.handle_name, self.door_name, self.goal_state_half_open, self.goal_state_full_open,
                           self.arms[0])


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be opened
        """
        arm: str
        """
        Arm that should be used for opening the container
        """

        @with_tree
        def perform(self) -> Any:
            # GraspingAction.Action(self.arm, self.object_designator).perform()
            OpeningMotion(self.object_designator, self.arm).resolve().perform()
            # MoveGripperMotion("open", "left").resolve().perform()
            # mvb = Pose([0,-0.2, 0],[0,0,0,1], "base_link")
            # NavigateAction([mvb]).resolve().perform()
            # ParkArmsAction([Arms.LEFT]).resolve().perform()
            # MoveGripperMotion("close", "left").resolve().perform()
            # giskard.finish_open_dishwasher_goal()
            # ParkArmsAction([Arms.LEFT]).resolve().perform()
            # MoveGripperMotion("open", self.arm, allow_gripper_collision=True).resolve().perform()

            # MoveGripperMotion("open", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMOpenAction:
            return ORMOpenAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMOpenAction:
            action = super().insert(session)

            op = self.object_designator.insert(session)
            action.object_id = op.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectPart, arms: List[str], resolver=None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter.
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be closed
        """
        arm: str
        """
        Arm that should be used for closing
        """

        @with_tree
        def perform(self) -> Any:
            GraspingAction.Action(self.arm, self.object_designator).perform()
            ClosingMotion(self.object_designator, self.arm).resolve().perform()

            MoveGripperMotion("open", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMCloseAction:
            return ORMCloseAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMCloseAction:
            action = super().insert(session)

            op = self.object_designator.insert(session)
            action.object_id = op.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectPart, arms: List[str], resolver=None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator description of the handle that should be used
        :param arms: A list of possible arms to use
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object designator and the first entry from
        the list of possible arms.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        arm: str
        """
        The arm that should be used to grasp
        """
        object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
        """
        Object Designator for the object that should be grasped
        """

        def perform(self) -> Any:
            # if isinstance(self.object_desig, ObjectPart.Object):
            object_pose = self.object_desig
            # else:
            #     object_pose = self.object_desig.bullet_world_object.get_pose()

            # Initialize the local transformer and robot reference
            lt = LocalTransformer()
            robot = BulletWorld.robot  # Retrieve object and robot from designators
            # Calculate the object's pose in the map frame
            oTm = object_pose
            # Todo only for suturo lab and hsr
            oTm.pose.position.x -= 0.2
            execute = True
            grasp = "front"
            # Determine the grasp orientation and transform the pose to the base link frame
            grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            # Set pose to the grasp rotation
            oTb.orientation = grasp_rotation

            object_orientation = axis_angle_to_quaternion([1, 0, 0], 90)
            q2 = [oTb.pose.orientation.x, oTb.pose.orientation.y,
                  oTb.pose.orientation.z, oTb.pose.orientation.w]
            new_qua = helper.multiply_quaternions(object_orientation, q2)

            oTb.pose.orientation.x = new_qua[0]
            oTb.pose.orientation.y = new_qua[1]
            oTb.pose.orientation.z = new_qua[2]
            oTb.pose.orientation.w = new_qua[3]

            tool_frame = robot_description.get_tool_frame(self.arm)
            oTgt = lt.transform_pose(oTb, robot.get_link_tf_frame(tool_frame))
            z = oTgt.pose.position.z
            oTgt.pose.position.z = z - 0.01
            oTmgt = lt.transform_pose(oTgt, "map")
            oTgt.pose.position.z = z - 0.01
            oTmG = lt.transform_pose(oTgt, "map")

            # Open the gripper before picking up the object
            rospy.logwarn("Opening Gripper")
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # Move to the pre-grasp position and visualize the action
            rospy.logwarn("Picking up now")

            BulletWorld.current_bullet_world.add_vis_axis(oTmgt)
            BulletWorld.current_bullet_world.add_vis_axis(oTmG)
            if execute:
                MoveTCPMotion(oTmgt, self.arm).resolve().perform()
                # MoveTCPMotion(oTmG, self.arm).resolve().perform()
            rospy.sleep(5)
            # Open the gripper before picking up the object
            rospy.logwarn("Closing Gripper")
            MoveGripperMotion(motion="close", gripper=self.arm).resolve().perform()

        def to_sql(self) -> ORMGraspingAction:
            return ORMGraspingAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMGraspingAction:
            action = super().insert(session)

            od = self.object_desig.insert(session)
            action.object_id = od.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, arms: List[str], object_description: Union[ObjectDesignatorDescription, ObjectPart],
                 resolver: Callable = None):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        :param resolver: An alternative resolver to get a specified designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[str] = arms
        self.object_description: ObjectDesignatorDescription = object_description

    def ground(self) -> Action:
        """
        Default resolver that takes the first element from the list of arms and the first solution for the object
        designator description ond returns it.

        :return: A performable action designator that contains specific arguments
        """
        return self.Action(self.arms[0], self.object_description.resolve())


class CuttingAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a cutting action.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Action class for the Cutting action.
        """

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be cut.
        """

        arm: str
        """
        The arm that should be used for cutting.
        """

        grasp: str
        """
        The grasp that should be used for cutting. For example, 'left' or 'right'.
        """

        slice_thickness: float
        """
        The upper bound thickness of the slices.
        """

        tool: str
        """
        The tool to cut with.
        """

        technique: str
        """
        Technique used to cut the object.
        """

        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False, repr=False)
        """
        The object at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        @with_tree
        def perform(self) -> None:
            """
            Perform the cutting action using the specified object, arm, grasp, slice thickness, tool, and technique.
            """
            # Store the object's data copy at execution
            self.object_at_execution = self.object_designator.data_copy()

            # Get grasp orientation and target pose
            grasp = robot_description.grasps.get_orientation_for_grasp(self.grasp)
            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object
            obj_dim = object.get_object_dimensions()

            dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
            oTm = object.get_pose()
            object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

            # from bread_dim calculate def a calculation that gets me the highest number from the first 2 entries
            # Given slice thickness is 3 cm or 0.03 meters
            slice_thickness = self.slice_thickness
            # Calculate slices and transform them to the map frame with orientation
            obj_length = dim[0]
            obj_width = dim[1]
            obj_height = dim[2]

            # Calculate the starting Y-coordinate offset (half the width minus half a slice thickness)
            if self.technique == 'halving':
                start_offset = 0
                num_slices = 1
            else:
                num_slices = 1
                # int(obj_length // slice_thickness))
                start_offset = 0  # -obj_length / 2 + slice_thickness / 2)

            # Calculate slice coordinates
            slice_coordinates = [start_offset + i * slice_thickness for i in range(num_slices)]

            # Transform slice coordinates to map frame with orientation
            slice_poses = []
            for x in slice_coordinates:
                tmp_pose = object_pose.copy()
                tmp_pose.pose.position.y -= 3 * obj_width
                tmp_pose.pose.position.x = x
                sTm = object.local_transformer.transform_pose(tmp_pose, "map")
                slice_poses.append(sTm)

            for slice_pose in slice_poses:
                # rotate the slice_pose by grasp
                ori = multiply_quaternions(
                    [slice_pose.orientation.x, slice_pose.orientation.y, slice_pose.orientation.z,
                     slice_pose.orientation.w], grasp)

                oriR = axis_angle_to_quaternion([0, 0, 1], 90)
                oriM = multiply_quaternions([oriR[0], oriR[1], oriR[2], oriR[3]], [ori[0], ori[1], ori[2], ori[3]])

                adjusted_slice_pose = slice_pose.copy()

                # Set the orientation of the object pose by grasp in MAP
                adjusted_slice_pose.orientation.x = oriM[0]
                adjusted_slice_pose.orientation.y = oriM[1]
                adjusted_slice_pose.orientation.z = oriM[2]
                adjusted_slice_pose.orientation.w = oriM[3]

                # Adjust the position of the object pose by grasp in MAP
                lift_pose = adjusted_slice_pose.copy()
                lift_pose.pose.position.z += 2 * obj_height
                # Perform the motion for lifting the tool
                BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()
                # Perform the motion for cutting the object
                BulletWorld.current_bullet_world.add_vis_axis(adjusted_slice_pose)
                MoveTCPMotion(adjusted_slice_pose, self.arm).resolve().perform()
                # Perform the motion for lifting the tool
                BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()

        # def to_sql(self) -> ORMCuttingAction:
        #     """
        #     Convert the action to a corresponding SQL representation for storage.
        #     """
        #     return ORMCuttingAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            """
            Insert the cutting action into the database session.
            """
            action = super().insert(session)
            # Additional logic for inserting cutting action data goes here
            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[str], grasps: List[str],
                 resolver=None):
        """
        Initialize the CuttingAction with object designators, arms, and grasps.

        :param object_designator_description: Object designator for the object to be cut.
        :param arms: List of possible arms that could be used.
        :param grasps: List of possible grasps for the cutting action.
        :param resolver: An optional resolver for dynamic parameter selection.
        """
        super(CuttingAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def __iter__(self):
        for object_, grasp, arm in itertools.product(iter(self.object_designator_description), self.grasps, self.arms):
            yield self.Action(object_, arm, grasp, slice_thickness=0.05, tool="big_knife", technique="slicing")

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return next(iter(self))


class PouringAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a pouring action.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Action class for the Pouring action.
        """

        target_location: Pose
        """
        The Pose the robot should pour into.
        """

        arm: str
        """
        The arm that should be used for pouring.
        """

        direction: str
        """
        The direction that should be used for pouring. For example, 'left' or 'right'.
        """

        angle: float
        """
        the angle to move the gripper to.
        """

        @with_tree
        def perform(self) -> None:

            # Initialize the local transformer and robot reference
            lt = LocalTransformer()
            robot = BulletWorld.robot
            # Retrieve object and robot from designators

            # Calculate the object's pose in the map frame
            oTm = self.target_location
            execute = True
            # BulletWorld.current_bullet_world.add_vis_axis(oTm)

            # Determine the grasp orientation and transform the pose to the base link frame
            grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
            oTbs = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            oTbs.pose.position.x += 0.009 #was 0,009
            oTbs.pose.position.z += 0.17 #was 0.13

            if self.direction == "right":
                oTbs.pose.position.y -= 0.125
            else:
                oTbs.pose.position.y += 0.125

            oTms = lt.transform_pose(oTbs, "map")
            BulletWorld.current_bullet_world.add_vis_axis(oTms)

            #
            oTog = lt.transform_pose(oTms, robot.get_link_tf_frame("base_link"))
            oTog.orientation = grasp_rotation
            oTgm = lt.transform_pose(oTog, "map")
            BulletWorld.current_bullet_world.add_vis_axis(oTgm)

            if self.direction == "right":
                new_q = axis_angle_to_quaternion([0, 0, 1], -self.angle)
            else:
                new_q = axis_angle_to_quaternion([0, 0, 1], self.angle)
            new_ori = multiply_quaternions(
                [oTgm.orientation.x, oTgm.orientation.y, oTgm.orientation.z,
                 oTgm.orientation.w], new_q)
            oTmsp = oTgm.copy()
            oTmsp.pose.orientation.x = new_ori[0]
            oTmsp.pose.orientation.y = new_ori[1]
            oTmsp.pose.orientation.z = new_ori[2]
            oTmsp.pose.orientation.w = new_ori[3]
            BulletWorld.current_bullet_world.add_vis_axis(oTmsp)

            if execute:
                 MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False).resolve().perform()
                 MoveTCPMotion(oTmsp, self.arm, allow_gripper_collision=False).resolve().perform()
                 MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False).resolve().perform()
            # # Move to the pre-grasp position and visualize the action
            # rospy.logwarn("Pre Pour")
            # BulletWorld.current_bullet_world.add_vis_axis(oTmso)
            #
            # #Execute Bool, because sometimes u only want to visualize the poses to pp.py things
            # if execute:
            #     MoveTCPMotion(oTmso, self.arm, allow_gripper_collision=False).resolve().perform()
            #
            #
            # if self.direction == "right":
            #     new_q = axis_angle_to_quaternion([0,0,1], -100)
            # else: new_q = axis_angle_to_quaternion([0,0,1], 100)
            # new_ori = multiply_quaternions(
            #     [oTmso.orientation.x, oTmso.orientation.y, oTmso.orientation.z,
            #      oTmso.orientation.w], new_q)
            # oTmsp = oTmso.copy()
            # oTmsp.pose.orientation.x = new_ori[0]
            # oTmsp.pose.orientation.y = new_ori[1]
            # oTmsp.pose.orientation.z = new_ori[2]
            # oTmsp.pose.orientation.w = new_ori[3]
            # BulletWorld.current_bullet_world.add_vis_axis(oTmsp)
            # if execute:
            #     MoveTCPMotion(oTmsp, self.arm, allow_gripper_collision=False).resolve().perform()
            #     MoveTCPMotion(oTmso, self.arm, allow_gripper_collision=False).resolve().perform()

            #     current_wrist_roll_joint = robot.get_joint_state('wrist_roll_joint')
            #
            #     # oTm = Object Pose in Frame map
            #     if self.direction == "right":
            #         oTm = Pose(
            #             [self.target_location.pose.position.x - 0.008, self.target_location.pose.position.y + 0.095,
            #              self.target_location.pose.position.z + 0.13], self.target_location.pose.orientation)
            #     else:
            #         oTm = Pose(
            #             [self.target_location.pose.position.x - 0.008, self.target_location.pose.position.y - 0.15,
            #              self.target_location.pose.position.z + 0.13], self.target_location.pose.orientation)
            #
            #     grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
            #     oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            #     oTb.orientation = grasp_rotation
            #     oTmG = lt.transform_pose(oTb, "map")
            #
            #     rospy.logwarn("Pouring now")
            #     MoveTorsoAction([0.37]).resolve().perform()
            #     MoveTCPMotion(oTmG, self.arm, allow_gripper_collision=False).resolve().perform()
            #
            #     PouringMotion(self.direction, self.angle - current_arm_roll_joint).resolve().perform()
            #     rospy.sleep(3)
            #
            #     if self.direction == "right":
            #         PouringMotion("left", current_wrist_roll_joint).resolve().perform()
            #     else:
            #         PouringMotion("right", current_wrist_roll_joint).resolve().perform()

    def __init__(self, target_locations: List[Pose], arms: List[str], directions: List[str], angles: List[float],
                 resolver=None):
        """
        :param target_locations: List of possible target locations to be poured into
        :param arms: List of possible arms that could be used
        :param directions: List of possible directions for the pouring direction
        :param angles: List of possible angles that the gripper tilts to
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of
                         possible paramter
        """
        super().__init__(resolver)
        self.target_locations: List[Pose] = target_locations
        self.arms: List[str] = arms
        self.directions: List[str] = directions
        self.angels: List[float] = angles

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible
        parameter.
        :return: A performable designator
        """
        return self.Action(self.target_locations[0], self.arms[0], self.directions[0], self.angels[0])


class MixingAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a mixing action.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Action class for the Mixing action.
        """

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be mixed.
        """

        object_tool_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the mixing tool.
        """

        arm: str
        """
        The arm that should be used for mixing.
        """

        grasp: str
        """
        The grasp that should be used for mixing. For example, 'left' or 'right'.
        """

        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        @with_tree
        def perform(self) -> None:
            """
            Perform the mixing action using the specified object, tool, arm, and grasp.
            """
            # Store the object's data copy at execution
            self.object_at_execution = self.object_designator.data_copy()
            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object

            obj_dim = object.get_object_dimensions()

            dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
            obj_height = dim[2]
            oTm = object.get_pose()
            object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

            def generate_spiral(pose, upward_increment, radial_increment, angle_increment, steps):
                x_start, y_start, z_start = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
                spiral_poses = []

                for t in range(2 * steps):
                    tmp_pose = pose.copy()

                    r = radial_increment * t
                    a = angle_increment * t
                    h = upward_increment * t

                    x = x_start + r * math.cos(a)
                    y = y_start + r * math.sin(a)
                    z = z_start + h

                    tmp_pose.pose.position.x += x
                    tmp_pose.pose.position.y += y
                    tmp_pose.pose.position.z += z

                    spiralTm = object.local_transformer.transform_pose(tmp_pose, "map")
                    spiral_poses.append(spiralTm)
                    BulletWorld.current_bullet_world.add_vis_axis(spiralTm)

                return spiral_poses

            # this is a very good one but takes ages
            # spiral_poses = generate_spiral(object_pose, 0.0004, 0.0008, math.radians(10), 100)
            spiral_poses = generate_spiral(object_pose, 0.001, 0.0035, math.radians(30), 10)

            BulletWorld.current_bullet_world.remove_vis_axis()
            for spiral_pose in spiral_poses:
                oriR = axis_angle_to_quaternion([1, 0, 0], 180)
                ori = multiply_quaternions(
                    [spiral_pose.orientation.x, spiral_pose.orientation.y, spiral_pose.orientation.z,
                     spiral_pose.orientation.w], oriR)
                adjusted_slice_pose = spiral_pose.copy()
                # # Set the orientation of the object pose by grasp in MAP
                adjusted_slice_pose.orientation.x = ori[0]
                adjusted_slice_pose.orientation.y = ori[1]
                adjusted_slice_pose.orientation.z = ori[2]
                adjusted_slice_pose.orientation.w = ori[3]

                # Adjust the position of the object pose by grasp in MAP
                lift_pose = adjusted_slice_pose.copy()
                lift_pose.pose.position.z += (obj_height + 0.08)
                # Perform the motion for lifting the tool
                # BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()

        # def to_sql(self) -> ORMMixingAction:
        #     """
        #     Convert the action to a corresponding SQL representation for storage.
        #     """
        #     return ORMMixingAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            """
            Insert the mixing action into the database session.
            """
            action = super().insert(session)
            # Additional logic for inserting mixing action data goes here
            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                 object_tool_designator_description: ObjectDesignatorDescription, arms: List[str], grasps: List[str],
                 resolver=None):
        """
        Initialize the MixingAction with object and tool designators, arms, and grasps.

        :param object_designator_description: Object designator for the object to be mixed.
        :param object_tool_designator_description: Object designator for the mixing tool.
        :param arms: List of possible arms that could be used.
        :param grasps: List of possible grasps for the mixing action.
        :param resolver: An optional resolver for dynamic parameter selection.
        """
        super(MixingAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.object_tool_designator_description: ObjectDesignatorDescription = object_tool_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(),
                           self.object_tool_designator_description.ground(), self.arms[0], self.grasps[0])


class HeadFollowAction(ActionDesignatorDescription):
    """
    Continuously move head to human closest to robot
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        state: str
        """
        defines if the robot should start/stop looking at human
        """

        @with_tree
        def perform(self) -> None:
            HeadFollowMotion(self.state).resolve().perform()

    def __init__(self, state: str, resolver=None):
        """
        :param state: defines if the robot should start/stop looking at human
        :param resolver: An optional resolver that returns a performable designator from the designator description
        """
        super().__init__(resolver)
        self.state = state

    def ground(self) -> Action:
        """
        Default resolver
        :return: A performable designator
        """
        return self.Action(self.state)


class DoorOpenAction(ActionDesignatorDescription):
    """
    grasp and open door
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        handle: str
        """
        defines the handle of the door to open
        """

        @with_tree
        def perform(self) -> None:
            MoveGripperMotion(motion="open", gripper="left").resolve().perform()
            GraspHandleMotion(self.handle).resolve().perform()
            MoveGripperMotion(motion="close", gripper="left").resolve().perform()
            DoorOpenMotion(self.handle).resolve().perform()
            MoveGripperMotion(motion="open", gripper="left").resolve().perform()

    def __init__(self, handle: str, resolver=None):
        """
        :param handle: handle in tf to grasp
        :param resolver: An optional resolver that returns a performable designator from the designator description
        """
        super().__init__(resolver)
        self.handle = handle

    def ground(self) -> Action:
        """
        Default resolver
        :return: A performable designator
        """
        return self.Action(self.handle)
