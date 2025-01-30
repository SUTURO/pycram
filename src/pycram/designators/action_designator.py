# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import inspect
import itertools
import math
from dataclasses import dataclass, field

import numpy as np
import rospy
import sqlalchemy
from geometry_msgs.msg import PointStamped
from giskardpy.data_types.exceptions import ForceTorqueThresholdException
from owlready2 import Thing
from sqlalchemy.orm import Session
from tf import transformations
from typing_extensions import List, Union, Callable, Optional, Type

from .location_designator import CostmapLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveArmJointsMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion, HeadFollowMotion, TalkingMotion, \
    MoveTCPForceTorqueMotion, GraspingDishwasherHandleMotion, HalfOpeningDishwasherMotion, MoveArmAroundMotion, \
    FullOpeningDishwasherMotion, MoveArmDownForceTorqueMotion
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..datastructures.enums import Arms, Grasp, GripperState, GiskardStateFTS
from ..datastructures.pose import Pose
from ..datastructures.world import World
from ..designator import ActionDesignatorDescription
from ..failures import ObjectUnfetchable, ReachabilityFailure, SensorMonitoringCondition, ManipulationFTSCheckNoObject
from ..helper import multiply_quaternions
from ..language import Monitor
from ..local_transformer import LocalTransformer
from ..luca_helper import adjust_grasp_for_object_rotation, calculate_object_faces
from ..ontology.ontology import OntologyConceptHolder
from ..orm.action_designator import Action as ORMAction
from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                     TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                     CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction, Action,
                                     FaceAtAction as ORMFaceAtAction)
from ..orm.base import Pose as ORMPose
from ..orm.object_designator import Object as ORMObject
from ..robot_description import RobotDescription
from ..ros.logging import logwarn
from ..ros_utils.force_torque_sensor import ForceTorqueSensor
from ..tasktree import with_tree
from ..utils import axis_angle_to_quaternion


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """

    def __init__(self, positions: List[float], resolver=None,
                 ontology_concept_holders: Optional[List[OntologyConceptHolder]] = None):
        """
        Create a designator description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        :param resolver: An optional specialized_designators that returns a performable designator for a designator description.
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.positions: List[float] = positions

        if self.soma:
            self.init_ontology_concepts({"move_torso": self.soma.MoveTorso})

    def ground(self) -> MoveTorsoActionPerformable:
        """
        Creates a performable action designator with the first element from the list of possible torso heights.

        :return: A performable action designator
        """
        return MoveTorsoActionPerformable(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator and returns a performable action designator with the value.

        :return: A performable action designator
        """
        for position in self.positions:
            yield MoveTorsoActionPerformable(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    def __init__(self, grippers: List[Arms], motions: List[GripperState], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        :param resolver: An alternative specialized_designators that returns a performable designator for a designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.grippers: List[GripperState] = grippers
        self.motions: List[Arms] = motions

        if self.soma:
            self.init_ontology_concepts({"setting_gripper": self.soma.SettingGripper})

    def ground(self) -> SetGripperActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first element in the grippers and motions list.

        :return: A performable designator
        """
        return SetGripperActionPerformable(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield SetGripperActionPerformable(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    def __init__(self, grippers: List[Arms], object_designator_description: ObjectDesignatorDescription,
                 resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        super().__init__(resolver, ontology_concept_holders)
        self.grippers: List[Arms] = grippers
        self.object_designator_description = object_designator_description

        if self.soma:
            self.init_ontology_concepts({"releasing": self.soma.Releasing})

    def ground(self) -> ReleaseActionPerformable:
        return ReleaseActionPerformable(self.grippers[0], self.object_designator_description.ground())


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    def __init__(self, grippers: List[Arms], object_designator_description: ObjectDesignatorDescription,
                 efforts: List[float], resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        super().__init__(resolver, ontology_concept_holders)
        self.grippers: List[Arms] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

        if self.soma:
            self.init_ontology_concepts({"holding": self.soma.Holding})

    def ground(self) -> GripActionPerformable:
        return GripActionPerformable(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    def __init__(self, arms: List[Arms], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        :param resolver: An optional specialized_designators that returns a performable designator from the designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.arms: List[Arms] = arms

        if self.soma:
            self.init_ontology_concepts({"parking_arms": self.soma.ParkingArms})

    def ground(self) -> ParkArmsActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        return ParkArmsActionPerformable(self.arms[0])


class PickUpAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[Arms], grasps: List[Grasp], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional specialized_designators that returns a performable designator with elements from the lists of possible paramter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.grasps: List[Grasp] = grasps

        if self.soma:
            self.init_ontology_concepts({"picking_up": self.soma.PickingUp})

    def ground(self) -> PickUpActionPerformable:
        """
        Default specialized_designators, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object):
            obj_desig = self.object_designator_description
        else:
            obj_desig = self.object_designator_description.resolve()

        return PickUpActionPerformable(obj_desig, self.arms[0], self.grasps[0])


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose], grasps: List[Grasp],
                 arms: List[Arms], with_force_torque: List[bool], resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param resolver: Grounding method to resolve this designator
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.target_locations: List[Pose] = target_locations
        if grasps:
            self.grasps: List[Grasp] = grasps
        self.arms: List[Arms] = arms

        if self.soma:
            self.init_ontology_concepts({"placing": self.soma.Placing})
        self.with_force_torque: List[bool] = with_force_torque

    def ground(self) -> PlaceActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return PlaceActionPerformable(obj_desig, self.arms[0], self.grasps[0], self.target_locations[0],
                                      self.with_force_torque[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    def __init__(self, target_locations: List[Pose], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative specialized_designators that creates a performable designator from the list of possible parameter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.target_locations: List[Pose] = target_locations

        if self.soma:
            self.init_ontology_concepts({"navigating": self.soma.Navigating})

    def ground(self) -> NavigateActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return NavigateActionPerformable(self.target_locations[0])


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[Arms],
                 target_locations: List[Pose], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator description or a specified Object designator that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative specialized_designators that returns a performable designator for the list of possible parameter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.target_locations: List[Pose] = target_locations

        if self.soma:
            self.init_ontology_concepts({"transporting": self.soma.Transporting})

    def ground(self) -> TransportActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description \
            if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object) \
            else self.object_designator_description.resolve()

        return TransportActionPerformable(obj_desig, self.arms[0], self.target_locations[0])


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    def __init__(self, targets: List[Pose], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative specialized_designators that returns a performable designator for a list of possible target locations
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.targets: List[Pose] = targets

        if self.soma:
            self.init_ontology_concepts({"looking_at": self.soma.LookingAt})

    def ground(self) -> LookAtActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return LookAtActionPerformable(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    def __init__(self, object_designator_description: Optional[ObjectDesignatorDescription] = None, technique: str = "all", state: Optional[str] = None,
                 resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator describing the object
        :param resolver: An alternative specialized_designators
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Optional[ObjectDesignatorDescription] = object_designator_description
        self.technique: Optional[str] = technique
        self.state: Optional[str] = state

        if not object_designator_description:
            obj_des = ObjectDesignatorDescription()
            self.object_designator_description = obj_des

        if self.soma:
            self.init_ontology_concepts({"looking_for": self.soma.LookingFor,
                                         "checking_object_presence": self.soma.CheckingObjectPresence})

    def ground(self) -> DetectActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the resolved object description.

        :return: A performable designator
        """

        return DetectActionPerformable(technique=self.technique, state=self.state, object_designator=self.object_designator_description.resolve())


class OpenDishwasherAction(ActionDesignatorDescription):
    """
    Opens the dishwasher door
    """

    def __init__(self, handle_name: str, door_name: str, goal_state_half_open: float, goal_state_full_open: float,
                 arms: List[Arms], resolver=None):
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
        self.arms: List[Arms] = arms

    def ground(self) -> OpenDishwasherPerformable:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return OpenDishwasherPerformable(self.handle_name, self.door_name, self.goal_state_half_open,
                                         self.goal_state_full_open, self.arms[0])


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative specialized_designators that returns a performable designator for the lists of possible parameter.
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms

        if self.soma:
            self.init_ontology_concepts({"opening": self.soma.Opening})

    def ground(self) -> OpenActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return OpenActionPerformable(self.object_designator_description.resolve(), self.arms[0])


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms],
                 resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator description of the handle that should be used
        :param arms: A list of possible arms to use
        :param resolver: An alternative specialized_designators that returns a performable designator for the list of possible parameter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms

        if self.soma:
            self.init_ontology_concepts({"closing": self.soma.Closing})

    def ground(self) -> CloseActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the resolved object designator and the first entry from
        the list of possible arms.

        :return: A performable designator
        """
        return CloseActionPerformable(self.object_designator_description.resolve(), self.arms[0])


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    def __init__(self, arms: List[Arms], object_description: Union[ObjectDesignatorDescription, ObjectPart],
                 resolver: Callable = None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        :param resolver: An alternative specialized_designators to get a specified designator from the designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.arms: List[Arms] = arms
        self.object_description: ObjectDesignatorDescription = object_description

        if self.soma:
            self.init_ontology_concepts({"grasping": self.soma.Grasping})

    def ground(self) -> GraspingActionPerformable:
        """
        Default specialized_designators that takes the first element from the list of arms and the first solution for the object
        designator description ond returns it.

        :return: A performable action designator that contains specific arguments
        """
        return GraspingActionPerformable(self.arms[0], self.object_description.resolve())


class HeadFollowAction(ActionDesignatorDescription):
    """
    Continuously move head to human closest to robot
    """

    def __init__(self, state: Optional[str], resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        :param state: defines if the robot should start/stop looking at human
        :param resolver: An optional resolver that returns a performable designator from the designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.state: Optional[str] = state

        if self.soma:
            self.init_ontology_concepts({"headfollow": self.soma.Headfollow})

    def ground(self) -> HeadFollowActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry
        in the list of possible targets

        :return: A performable designator
        """
        return HeadFollowActionPerformable(self.state)


class PouringAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a pouring action.
    """

    target_location: Pose
    """
    The Pose the robot should pour into.
    """

    arm: Arms
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

    def __init__(self, target_locations: List[Pose], arms: List[Arms], directions: List[str], angles: List[float],
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
        self.arms: List[Arms] = arms
        self.directions: List[str] = directions
        self.angels: List[float] = angles

    def ground(self) -> PouringActionPerformable:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible
        parameter.
        :return: A performable designator
        """
        return PouringActionPerformable(self.target_locations[0], self.arms[0], self.directions[0], self.angels[0])


class MixingAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a mixing action.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be mixed.
    """

    object_tool_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the mixing tool.
    """

    arm: Arms
    """
    The arm that should be used for mixing.
    """

    grasp: Grasp
    """
    The grasp that should be used for mixing. For example, 'left' or 'right'.
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """
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
                 object_tool_designator_description: ObjectDesignatorDescription, arms: List[Arms], grasps: List[Grasp],
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
        self.arms: List[Arms] = arms
        self.grasps: List[Grasp] = grasps

    def ground(self) -> MixingActionPerformable:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.
        :return: A performable designator
        """
        return MixingActionPerformable(self.object_designator_description.ground(),
                                       self.object_tool_designator_description.ground(), self.arms[0], self.grasps[0])


class PlaceGivenObjectAction(ActionDesignatorDescription):
    """
       A class representing a designator for a place action of human given objects, allowing a robot to place a
     human given object, that could not be picked up or were not found in the FOV.
    This class encapsulates the details of the place action of human given objects, including the type of the object to
    be placed, the arm to be used, the target_location to place the object and the grasp type. It defines the sequence
    of operations for the robot to execute the place action of human given object, such as moving the arm holding the
    object to the target_location, opening the gripper, and lifting the arm.
    """
    object_type: str
    """
    Object type describing the object that should be placed
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """

    grasp: Grasp
    """
    Grasp that defines how to place the given object
    """

    on_table: Optional[bool]
    """
    When placing a plate needed to differentiate between placing in a dishwasher and placing on the table. 
    Default is placing on a table.
    """

    def __init__(self,
                 object_types: List[str], arms: List[Arms], target_locations: List[Pose], grasps: List[Grasp],
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
        self.arms: List[Arms] = arms
        self.grasps: List[Grasp] = grasps
        self.target_locations: List[Pose] = target_locations
        self.on_table: bool = on_table

    def ground(self) -> PlaceGivenObjectPerformable:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible
        parameter.
        :return: A performable designator
        """
        return PlaceGivenObjectPerformable(self.object_types[0], self.arms[0], self.target_locations[0], self.grasps[0], self.on_table)

# ----------------------------------------------------------------------------
# ---------------- Performables ----------------------------------------------
# ----------------------------------------------------------------------------


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable performables."""
    orm_class: Type[ORMAction] = field(init=False, default=None)
    """
    The ORM class that is used to insert this action into the database. Must be overwritten by every action in order to
    be able to insert the action into the database.
    """

    @abc.abstractmethod
    def perform(self) -> None:
        """
        Perform the action.

        Will be overwritten by each action.
        """
        pass

    def to_sql(self) -> Action:
        """
        Convert this action to its ORM equivalent.

        Needs to be overwritten by an action if it didn't overwrite the orm_class attribute with its ORM equivalent.

        :return: An instance of the ORM equivalent of the action with the parameters set
        """
        # get all class parameters
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # list of parameters that will be passed to the ORM class. If the name does not match the orm_class equivalent
        # or if it is a type that needs to be inserted into the session manually, it will not be added to the list
        parameters = [value for key, value in class_variables.items() if key in orm_class_variables
                      and not isinstance(value, (ObjectDesignatorDescription.Object, Pose))]

        return self.orm_class(*parameters)

    def insert(self, session: Session, **kwargs) -> Action:
        """
        Insert this action into the database.

        Needs to be overwritten by an action if the action has attributes that do not exist in the orm class
        equivalent. In that case, the attributes need to be inserted into the session manually.

        :param session: Session with a database that is used to add and commit the objects
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM action that was inserted into the database
        """

        action = super().insert(session)

        # get all class parameters
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # loop through all class parameters and insert them into the session unless they are already added by the ORM
        for key, value in class_variables.items():
            if key not in orm_class_variables:
                variable = value.insert(session)
                if isinstance(variable, ORMObject):
                    action.object = variable
                elif isinstance(variable, ORMPose):
                    action.pose = variable
        session.add(action)

        return action


@dataclass
class MoveTorsoActionPerformable(ActionAbstract):
    """
    Move the torso of the robot up and down.
    """

    position: float
    """
    Target position of the torso joint
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMMoveTorsoAction)

    @with_tree
    def perform(self) -> None:
        MoveJointsMotion([RobotDescription.current_robot_description.torso_joint], [self.position]).perform()


@dataclass
class SetGripperActionPerformable(ActionAbstract):
    """
    Set the gripper state of the robot.
    """

    gripper: Arms
    """
    The gripper that should be set 
    """
    motion: GripperState
    """
    The motion that should be set on the gripper
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMSetGripperAction)

    @with_tree
    def perform(self) -> None:
        MoveGripperMotion(gripper=self.gripper, motion=self.motion).perform()


@dataclass
class ReleaseActionPerformable(ActionAbstract):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """

    gripper: Arms

    object_designator: ObjectDesignatorDescription.Object

    def perform(self) -> None:
        raise NotImplementedError


@dataclass
class GripActionPerformable(ActionAbstract):
    """
    Grip an object with the robot.

    Note: This action can not be used yet.
    """

    gripper: Arms
    object_designator: ObjectDesignatorDescription.Object
    effort: float

    @with_tree
    def perform(self) -> None:
        raise NotImplementedError()


@dataclass
class ParkArmsActionPerformable(ActionAbstract):
    """
    Park the arms of the robot.
    """

    arm: Arms
    """
    Entry from the enum for which arm should be parked
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMParkArmsAction)

    @with_tree
    def perform(self) -> None:
        # create the keyword arguments
        kwargs = dict()
        left_poses = None
        right_poses = None

        # add park left arm if wanted
        if self.arm in [Arms.LEFT, Arms.BOTH]:
            kwargs["left_arm_config"] = "park"
            left_poses = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).get_static_joint_states(
                kwargs["left_arm_config"])

        # add park right arm if wanted
        if self.arm in [Arms.RIGHT, Arms.BOTH]:
            kwargs["right_arm_config"] = "park"
            right_poses = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_static_joint_states(
                kwargs["right_arm_config"])

        MoveArmJointsMotion(left_poses, right_poses).perform()


@dataclass
class PickUpActionPerformable(ActionAbstract):
    """
    Let the robot pick up an object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be picked up
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp: Grasp
    """
    The grasp that should be used. For example, 'left' or 'right'
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPickUpAction)

    def __post_init__(self):
        super(ActionAbstract, self).__post_init__()
        # Store the object's data copy at execution
        self.object_at_execution = self.object_designator.frozen_copy()

    @with_tree
    def perform(self) -> None:
        # Initialize the local transformer and robot reference
        lt = LocalTransformer()
        robot = World.robot
        # Retrieve object and robot from designators
        object = self.object_designator.world_object
        # Calculate the object's pose in the map frame
        oTm = object.get_pose()
        execute = True

        # Adjust object pose for top-grasping, if applicable
        if self.grasp == Grasp.TOP:
            print("Metalbowl from top")
            # Handle special cases for certain object types (e.g., Cutlery, Metalbowl)
            # Note: This includes hardcoded adjustments and should ideally be generalized
            # if self.object_designator.type == "Cutlery":
            # todo: this z is the popcorn-table height, we need to define location to get that z otherwise it
            #  is hardcoded
            # oTm.pose.position.z = 0.71
            oTm.pose.position.z += 0.035

        # Determine the grasp orientation and transform the pose to the base link frame

        grasp_rotation = RobotDescription.current_robot_description.grasps[self.grasp]
        oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        # Set pose to the grasp rotation
        oTb.orientation = grasp_rotation
        # Transform the pose to the map frame
        oTmG = lt.transform_pose(oTb, "map")

        # Open the gripper before picking up the object
        rospy.logwarn("Opening Gripper")
        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        # Move to the pre-grasp position and visualize the action
        rospy.logwarn("Picking up now")
        World.current_world.add_vis_axis(oTmG)
        # Execute Bool, because sometimes u only want to visualize the poses to pp.py things
        if execute:
            MoveTCPMotion(oTmG, self.arm, allow_gripper_collision=False).perform()

        # Calculate and apply any special knowledge offsets based on the robot and object type
        # Note: This currently includes robot-specific logic that should be generalized
        tool_frame = RobotDescription.current_robot_description.get_arm_tool_frame(self.arm)
        special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))

        # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
        #  depending on robot
        if robot.name == "hsrb":
            if self.grasp == Grasp.TOP:
                if self.object_designator.obj_type in ["Spoon", "Fork", "Knife", "Plasticknife"]:
                    special_knowledge_offset.pose.position.y -= 0.02
                if self.object_designator.obj_type == "Metalbowl":
                    special_knowledge_offset.pose.position.y -= 0.085
                    special_knowledge_offset.pose.position.x += 0.03

        push_base = special_knowledge_offset
        # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
        #  depending on robot if we dont generlize this we will have a big list in the end of all robots
        if robot.name == "hsrb":
            z = 0.04
            if self.grasp == Grasp.TOP:
                z = 0.025
                if self.object_designator.obj_type == "Metalbowl":
                    z = 0.035
            push_base.pose.position.z += z
        push_baseTm = lt.transform_pose(push_base, "map")
        special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")

        # Grasping from the top inherently requires calculating an offset, whereas front grasping involves
        # slightly pushing the object forward.
        rospy.logwarn("Offset now")
        # m = ManualMarkerPublisher()
        # m.create_marker("pose_pickup", special_knowledge_offsetTm)
        World.current_world.add_vis_axis(special_knowledge_offsetTm)
        if execute:
            MoveTCPMotion(special_knowledge_offsetTm, self.arm, allow_gripper_collision=False).perform()

        rospy.logwarn("Pushing now")
        World.current_world.add_vis_axis(push_baseTm)
        if execute:
            MoveTCPMotion(push_baseTm, self.arm, allow_gripper_collision=False).perform()

        # Finalize the pick-up by closing the gripper and lifting the object
        rospy.logwarn("Close Gripper")
        MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm, allow_gripper_collision=True).perform()

        rospy.logwarn("Lifting now")
        liftingTm = push_baseTm
        liftingTm.pose.position.z += 0.03
        World.current_world.add_vis_axis(liftingTm)
        if execute:
            if self.object_designator.obj_type in ["Spoon", "Fork", "Knife", "Plasticknife", "Cutlery"]:
                MoveTCPMotion(liftingTm, self.arm, allow_gripper_collision=False).perform()
            else:
                if self.object_designator.obj_type != "Metalbowl":
                    object_type = "Default"
                else:
                    object_type = "Bowl"
                try:
                    MoveTCPForceTorqueMotion(liftingTm, Arms.LEFT, object_type, GiskardStateFTS.GRASP,
                                             allow_gripper_collision=False).perform()
                except ForceTorqueThresholdException:
                    raise ManipulationFTSCheckNoObject(f"Could not pickup object after checking force-torque values")
        tool_frame = RobotDescription.current_robot_description.get_arm_tool_frame(arm=self.arm)
        robot.attach(child_object=self.object_designator.world_object, parent_link=tool_frame)

    # TODO find a way to use object_at_execution instead of object_designator in the automatic orm mapping in ActionAbstract
    def to_sql(self) -> Action:
        return ORMPickUpAction(arm=self.arm, grasp=self.grasp)

    def insert(self, session: Session, **kwargs) -> Action:
        action = super(ActionAbstract, self).insert(session)
        action.object = self.object_at_execution.insert(session)

        session.add(action)
        return action


@dataclass
class PlaceActionPerformable(ActionAbstract):
    """
    Places an Object at a position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be place
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """
    grasp: Grasp
    """
    Grasp that was used to pick up the object
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPlaceAction)

    """
    Pose in the world at which the object should be placed
    """
    with_force_torque: bool = True

    @with_tree
    def perform(self) -> None:
        lt = LocalTransformer()
        execute = True
        robot = World.robot
        oTm = self.target_location

        if self.with_force_torque:
            if self.grasp == Grasp.TOP:
                oTm.pose.position.z += 0.12
            else:
                oTm.pose.position.z += 0.2
        else:
            if self.grasp == Grasp.TOP:
                oTm.pose.position.z += 0.05

        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = RobotDescription.current_robot_description.grasps[self.grasp]

        oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        # Set pose to the grasp rotation
        oTb.orientation = grasp_rotation
        # Transform the pose to the map frame
        oTmG = lt.transform_pose(oTb, "map")

        rospy.logwarn("Placing now")
        World.current_world.add_vis_axis(oTmG)
        if execute:
            MoveTCPMotion(oTmG, self.arm).perform()

        if self.with_force_torque:
            if self.object_designator.obj_type != "Metalbowl":
                object_type = "Default"
            else:
                object_type = "Bowl"
            try:
                MoveArmDownForceTorqueMotion(down_distance=0.3, object_type=object_type, speed_multi=0.1)
            except ForceTorqueThresholdException:
                raise ManipulationFTSCheckNoObject(f"Could not place object after checking force-torque values")
        else:
            tool_frame = RobotDescription.current_robot_description.get_arm_tool_frame(self.arm)
            push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
            if robot.name == "hsrb":
                z = 0.03
                if self.grasp == Grasp.TOP:
                    z = 0.07
                push_base.pose.position.z += z
            # todo: make this for other robots
            push_baseTm = lt.transform_pose(push_base, "map")

            rospy.logwarn("Pushing now")
            World.current_world.add_vis_axis(push_baseTm)
            if execute:
                MoveTCPMotion(push_baseTm, self.arm).perform()

            if self.object_designator.obj_type == "Metalplate":
                loweringTm = push_baseTm
                loweringTm.pose.position.z -= 0.08
                World.current_world.add_vis_axis(loweringTm)
                if execute:
                    MoveTCPMotion(loweringTm, self.arm).perform()
                # rTb = Pose([0,-0.1,0], [0,0,0,1],"base_link")
                rospy.logwarn("sidepush monitoring")
                TalkingMotion("sidepush.").perform()
                side_push = Pose(
                    [push_baseTm.pose.position.x, push_baseTm.pose.position.y + 0.08, push_baseTm.pose.position.z],
                    [push_baseTm.orientation.x, push_baseTm.orientation.y, push_baseTm.orientation.z,
                     push_baseTm.orientation.w])
                try:
                    plan = MoveTCPMotion(side_push, self.arm) >> Monitor(monitor_func)
                    plan.perform()
                except SensorMonitoringCondition:
                    rospy.logwarn("Open Gripper")
                    MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        # Finalize the placing by opening the gripper and lifting the arm
        rospy.logwarn("Open Gripper")
        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()
        robot.detach(self.object_designator.world_object)
        rospy.logwarn("Lifting now")
        liftingTm = oTmG
        liftingTm.pose.position.z += 0.08
        World.current_world.add_vis_axis(liftingTm)
        if execute:
            MoveTCPMotion(liftingTm, self.arm).perform()

@dataclass
class NavigateActionPerformable(ActionAbstract):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Location to which the robot should be navigated
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMNavigateAction)

    @with_tree
    def perform(self) -> None:
        MoveMotion(self.target_location).perform()


@dataclass
class TransportActionPerformable(ActionAbstract):
    """
    Transports an object to a position using an arm
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be transported.
    """
    arm: Arms
    """
    Arm that should be used
    """
    target_location: Pose
    """
    Target Location to which the object should be transported
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMTransportAction)

    @with_tree
    def perform(self) -> None:
        robot_desig = BelieveObject(names=[RobotDescription.current_robot_description.name])
        ParkArmsActionPerformable(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=self.object_designator, reachable_for=robot_desig.resolve(),
                                     reachable_arm=self.arm)
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = None
        for pose in pickup_loc:
            if self.arm in pose.reachable_arms:
                pickup_pose = pose
                break
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateActionPerformable(pickup_pose.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, Grasp.FRONT).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig.resolve(),
                                        reachable_arm=self.arm).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                f"No location found from where the robot can reach the target location: {self.target_location}")
        NavigateActionPerformable(place_loc.pose).perform()
        PlaceActionPerformable(self.object_designator, self.arm, self.target_location).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()


@dataclass
class LookAtActionPerformable(ActionAbstract):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        LookingMotion(target=self.target).perform()


@dataclass
class DetectActionPerformable(ActionAbstract):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """
    technique: str
    """
    Technique means how the object should be detected, e.g. 'color', 'shape', 'region', etc. 
    Or 'all' if all objects should be detected
    """
    state: Optional[str]
    """
    Technique means how the object should be detected, e.g. 'color', 'shape', 'region', etc. 
    Or 'all' if all objects should be detected
    """

    object_designator: ObjectDesignatorDescription.Object = None
    """
    Object designator loosely describing the object, e.g. only type. 
    """

    state: Optional[str] = None
    """
    The state instructs our perception system to either start or stop the search for an object or human.
    Can also be used to describe the region or location where objects are perceived.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMDetectAction)

    @with_tree
    def perform(self) -> None:
        return DetectingMotion(object_type=self.object_designator.obj_type, technique=self.technique,
                               state=self.state).perform()


@dataclass
class OpenDishwasherPerformable(ActionAbstract):
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

    arm: Arms
    """
    Arm that should be used for opening the container
    """

    @with_tree
    def perform(self) -> None:
        # Grasping the dishwasher handle
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        GraspingDishwasherHandleMotion(self.handle_name, self.arm).perform()

        # partially opening the dishwasher door
        MoveGripperMotion(GripperState.CLOSE, self.arm).perform()
        HalfOpeningDishwasherMotion(self.handle_name, self.goal_state_half_open, self.arm).perform()

        # moves arm around the door to further push it open
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        MoveArmAroundMotion(self.handle_name, self.arm).perform()

        # pushes the rest of the door open
        MoveGripperMotion(GripperState.CLOSE, self.arm).perform()
        FullOpeningDishwasherMotion(self.handle_name, self.door_name, self.goal_state_full_open,
                                    self.arm).perform()

        ParkArmsAction([self.arm]).resolve().perform()
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        # plan = talk | park | gripper_open
        # plan.perform()


@dataclass
class OpenActionPerformable(ActionAbstract):
    """
    Opens a container like object
    """

    object_designator: ObjectPart.Object
    """
    Object designator describing the object that should be opened
    """
    arm: Arms
    """
    Arm that should be used for opening the container
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMOpenAction)

    @with_tree
    def perform(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        OpeningMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()


@dataclass
class CloseActionPerformable(ActionAbstract):
    """
    Closes a container like object.
    """

    object_designator: ObjectPart.Object
    """
    Object designator describing the object that should be closed
    """
    arm: Arms
    """
    Arm that should be used for closing
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMCloseAction)

    @with_tree
    def perform(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator).perform()
        ClosingMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()


@dataclass
class GraspingActionPerformable(ActionAbstract):
    """
    Grasps an object described by the given Object Designator description
    """
    arm: Arms
    """
    The arm that should be used to grasp
    """
    object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
    """
    Object Designator for the object that should be grasped
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMGraspingAction)

    @with_tree
    def perform(self) -> None:
        if isinstance(self.object_desig, ObjectPart.Object):
            object_pose = self.object_desig.part_pose
        else:
            object_pose = self.object_desig.world_object.get_pose()
        lt = LocalTransformer()
        gripper_name = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()

        object_pose_in_gripper = lt.transform_pose(object_pose,
                                                   World.robot.get_link_tf_frame(gripper_name))

        pre_grasp = object_pose_in_gripper.copy()
        pre_grasp.pose.position.x -= 0.1

        MoveTCPMotion(pre_grasp, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()

        MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion(GripperState.CLOSE, self.arm, allow_gripper_collision=True).perform()


@dataclass
class FaceAtPerformable(ActionAbstract):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose: Pose
    """
    The pose to face 
    """

    orm_class = ORMFaceAtAction

    @with_tree
    def perform(self) -> None:
        # get the robot position
        robot_position = World.robot.pose

        # calculate orientation for robot to face the object
        angle = np.arctan2(robot_position.position.y - self.pose.position.y,
                           robot_position.position.x - self.pose.position.x) + np.pi
        orientation = list(transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))

        # create new robot pose
        new_robot_pose = Pose(robot_position.position_as_list(), orientation)

        # turn robot
        NavigateActionPerformable(new_robot_pose).perform()

        # look at target
        LookAtActionPerformable(self.pose).perform()


@dataclass
class MoveAndPickUpPerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    arm: Arms
    """
    The arm to use
    """

    grasp: Grasp
    """
    The grasp to use
    """

    @with_tree
    def perform(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.object_designator.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, self.grasp).perform()


@dataclass
class MoveAndPlacePerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    target_location: Pose
    """
    The location to place the object.
    """

    arm: Arms
    """
    The arm to use
    """

    @with_tree
    def perform(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.target_location).perform()
        PlaceActionPerformable(self.object_designator, self.arm, self.target_location).perform()


@dataclass
class HeadFollowActionPerformable(ActionAbstract):
    """
    Continuously move head to human closest to robot
    """

    state: Optional[str]
    """
    defines if the robot should start/stop looking at human
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        HeadFollowMotion(self.state).perform()


@dataclass
class PouringActionPerformable(ActionAbstract):
    """
    Designator to let the robot perform a pouring action.
    """

    target_location: Pose
    """
    The Pose the robot should pour into.
    """

    arm: Arms
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
    def perform(self):
        # Initialize the local transformer and robot reference
        lt = LocalTransformer()
        robot = World.robot

        # Calculate the object's pose in the map frame
        oTm = self.target_location
        execute = True

        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = RobotDescription.current_robot_description.grasps[Grasp.FRONT]
        oTbs = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        oTbs.pose.position.x += 0.009  # was 0,009
        oTbs.pose.position.z += 0.17  # was 0.13

        if self.direction == "right":
            oTbs.pose.position.y -= 0.125
        else:
            oTbs.pose.position.y += 0.125

        oTms = lt.transform_pose(oTbs, "map")
        World.current_world.add_vis_axis(oTms)

        #
        oTog = lt.transform_pose(oTms, robot.get_link_tf_frame("base_link"))
        oTog.orientation = grasp_rotation
        oTgm = lt.transform_pose(oTog, "map")
        World.current_world.add_vis_axis(oTgm)

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
        World.current_world.add_vis_axis(oTmsp)

        if execute:
            MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False).perform()
            MoveTCPMotion(oTmsp, self.arm, allow_gripper_collision=False).perform()
            MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False).perform()


@dataclass
class MixingActionPerformable(ActionAbstract):

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be mixed.
    """

    object_tool_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the mixing tool.
    """

    arm: Arms
    """
    The arm that should be used for mixing.
    """

    grasp: Grasp
    """
    The grasp that should be used for mixing. For example, 'left' or 'right'.
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)
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
        object = self.object_designator.world_object

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
                World.current_world.add_vis_axis(spiralTm)

            return spiral_poses

        # this is a very good one but takes ages
        # spiral_poses = generate_spiral(object_pose, 0.0004, 0.0008, math.radians(10), 100)
        spiral_poses = generate_spiral(object_pose, 0.001, 0.0035, math.radians(30), 10)

        World.current_world.remove_vis_axis()
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
            MoveTCPMotion(lift_pose, self.arm).perform()

    # def to_sql(self) -> ORMMixingAction:
    #     """
    #     Convert the action to a corresponding SQL representation for storage.
    #     """
    #     return ORMMixingAction(self.arm, self.grasp)


@dataclass
class PlaceGivenObjectPerformable(ActionAbstract):
    object_type: str
    """
    Object type describing the object that should be placed
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """

    grasp: Grasp
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
        robot = World.robot
        fts = ForceTorqueSensor(robot_name=robot.name)

        # oTm = Object Pose in Frame map
        oTm = self.target_location
        execute = True

        # TODO add for other robots
        if self.object_type == "Metalplate" and self.on_table and robot.name == "hsrb":

            grasp_rotation = RobotDescription.current_robot_description.grasps[Grasp.FRONT]
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            oTb.orientation = grasp_rotation
            oTmG = lt.transform_pose(oTb, "map")

            logwarn("Placing now")
            MoveTCPMotion(oTmG, self.arm).perform()

            MoveTorsoAction([0.62]).resolve().perform()
            kwargs = dict()

            # taking in the predefined arm configuration for placing
            if self.arm in [Arms.LEFT, Arms.BOTH]:
                kwargs["left_arm_config"] = "place_plate"
                MoveArmJointsMotion(**kwargs).perform()

            # turning the gripper downwards to better drop the plate
            MoveJointsMotion(["wrist_flex_joint"], [-0.8]).perform()

            # correct a possible sloped orientation
            NavigateAction(
                [Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y,
                       0])]).resolve().perform()

            MoveGripperMotion(motion=GripperState.OPEN, gripper=Arms.LEFT).perform()

            # Move away from the table
            # todo generalize so that hsr is always moving backwards
            NavigateAction(
                [Pose([robot.get_pose().pose.position.x - 0.1, robot.get_pose().pose.position.y,
                       0])]).resolve().perform()

        # placing everything else or the Metalplate in the dishwasher
        else:
            if self.grasp == Grasp.TOP:
                oTm.pose.position.z += 0.05

            # Determine the grasp orientation and transform the pose to the base link frame
            grasp_rotation = RobotDescription.current_robot_description.grasps[self.grasp]
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            # Set pose to the grasp rotation
            oTb.orientation = grasp_rotation
            # Transform the pose to the map frame
            oTmG = lt.transform_pose(oTb, "map")

            logwarn("Placing now")
            World.current_world.add_vis_axis(oTmG)
            if execute:
                MoveTCPMotion(oTmG, self.arm).perform()

            tool_frame = RobotDescription.current_robot_description.get_arm_tool_frame(self.arm)
            push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
            if robot.name == "hsrb":
                z = 0.03
                if self.grasp == Grasp.TOP:
                    z = 0.07
                push_base.pose.position.z += z
            # todo: make this for other robots
            push_baseTm = lt.transform_pose(push_base, "map")

            logwarn("Pushing now")
            World.current_world.add_vis_axis(push_baseTm)
            if execute:
                MoveTCPMotion(push_baseTm, self.arm).perform()
            if self.object_type == "Metalplate":
                loweringTm = push_baseTm
                loweringTm.pose.position.z -= 0.08
                World.current_world.add_vis_axis(loweringTm)
                if execute:
                    MoveTCPMotion(loweringTm, self.arm).perform()
                # rTb = Pose([0,-0.1,0], [0,0,0,1],"base_link")
                logwarn("sidepush monitoring")
                TalkingMotion("sidepush.").perform()
                side_push = Pose(
                    [push_baseTm.pose.position.x, push_baseTm.pose.position.y + 0.125, loweringTm.pose.position.z],
                    [push_baseTm.orientation.x, push_baseTm.orientation.y, push_baseTm.orientation.z,
                     push_baseTm.orientation.w])
                try:
                    plan = MoveTCPMotion(side_push, self.arm) >> Monitor(fts.monitor_func)
                    plan.perform()
                except (SensorMonitoringCondition):
                    logwarn("Open Gripper")
                    MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

            # Finalize the placing by opening the gripper and lifting the arm
            logwarn("Open Gripper")
            MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

            logwarn("Lifting now")
            liftingTm = push_baseTm
            liftingTm.pose.position.z += 0.08
            World.current_world.add_vis_axis(liftingTm)
            if execute:
                MoveTCPMotion(liftingTm, self.arm).perform()
