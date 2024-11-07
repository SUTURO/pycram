# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import inspect
import itertools

import numpy as np
from sqlalchemy.orm import Session
from tf import transformations
from typing_extensions import Any, List, Union, Callable, Optional, Type

import rospy

from .location_designator import CostmapLocation
from .motion_designator import *
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..local_transformer import LocalTransformer
from ..plan_failures import ObjectUnfetchable, ReachabilityFailure
# from ..robot_descriptions import robot_description
from ..robot_description import RobotDescription
from ..tasktree import with_tree

from owlready2 import Thing

from ..datastructures.enums import Arms, Grasp, GripperState
from ..designator import ActionDesignatorDescription
from ..datastructures.pose import Pose
from ..datastructures.world import World
from ..ontology.ontology import OntologyConceptHolder

from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                     TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                     CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction, Action,
                                     FaceAtAction as ORMFaceAtAction)
from ..orm.base import Pose as ORMPose
from ..orm.object_designator import Object as ORMObject
from ..orm.action_designator import Action as ORMAction
from dataclasses import dataclass, field

from ..utilities.robocup_utils import multiply_quaternions


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
        self.grippers: List[Arms] = grippers
        self.motions: List[GripperState] = motions

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
                 object_designator_description: List[ObjectDesignatorDescription.Object],
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
        self.object_designator_description: List[ObjectDesignatorDescription.Object] = object_designator_description
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
            print("##############")
            print(type(self.object_designator_description[0]))
            print("##############")
            obj_desig = self.object_designator_description

        return PickUpActionPerformable(obj_desig[0], self.arms[0], self.grasps[0])


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose], grasp: Grasp,
                 arms: List[Arms], resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param grasp: grasp orientation
        :param resolver: Grounding method to resolve this designator
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.target_locations: List[Pose] = target_locations
        self.arms: List[Arms] = arms
        self.grasp = grasp

        if self.soma:
            self.init_ontology_concepts({"placing": self.soma.Placing})

    def ground(self) -> PlaceActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return PlaceActionPerformable(obj_desig, self.arms[0], self.grasp, self.target_locations[0])


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

    def __init__(self, object_designator_description: Optional[ObjectDesignatorDescription] = None, resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None, state: Optional[str] = None, technique: Optional[str] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator describing the object
        :param resolver: An alternative specialized_designators
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        :param state: for special instructions e.g. "stop" for stop detecting
        :param technique: what should be detected e.g. "human", "bowl", "object"
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_designator_description: Optional[ObjectDesignatorDescription] = object_designator_description
        self.state: Optional[str] = state
        self.technique: Optional[str] = technique

        if self.soma:
            self.init_ontology_concepts({"looking_for": self.soma.LookingFor,
                                         "checking_object_presence": self.soma.CheckingObjectPresence})

    def ground(self) -> DetectActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the resolved object description.

        :return: A performable designator
        """
        if self.object_designator_description:
            return DetectActionPerformable(object_designator=self.object_designator_description.resolve(),
                                           state=self.state, technique=self.technique)
        else:
            place_holder = ObjectDesignatorDescription()
            self.object_designator_description = place_holder
            return DetectActionPerformable(object_designator=self.object_designator_description.resolve(),
                                           state=self.state, technique=self.technique)


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


class PointingAction(ActionDesignatorDescription):
    """
    Point to Pose
    """
    def __init__(self, x_coordinate: float, y_coordinate: float, z_coordinate: float, resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Lets the robot pour based on the given parameter.
        :param x_coordinate: x coordinate where the robot points to (in map frame)
        :param y_coordinate: y coordinate where the robot points to (in map frame)
        :param z_coordinate: z coordinate where the robot points to (in map frame)
        :param resolver: An alternative resolver
        """
        super().__init__(resolver, ontology_concept_holders)
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate
        self.z_coordinate = z_coordinate

        if self.soma:
            self.init_ontology_concepts({"pointing": self.soma.Headfollow})

    def ground(self) -> PointingActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry
        in the list of possible targets

        :return: A performable designator
        """
        return PointingActionPerformable(self.x_coordinate, self.y_coordinate, self.z_coordinate)


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
        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
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

        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
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

    Note: This action can not be used yet.
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
            left_poses = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).get_static_joint_states(kwargs["left_arm_config"])

        # add park right arm if wanted
        if self.arm in [Arms.RIGHT, Arms.BOTH]:
            kwargs["right_arm_config"] = "park"
            right_poses = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_static_joint_states(kwargs["right_arm_config"])

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

    @with_tree
    def perform(self) -> None:
        print("in Performable")
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
            # oTm.pose.position.z += 0.035
            oTm.pose.position.z -= 0.035

        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = RobotDescription.current_robot_description.grasps[self.grasp]
        oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        # Set pose to the grasp rotation
        oTb.orientation = multiply_quaternions(oTb.orientation_as_list(), grasp_rotation)  # [0, 0, 0, 1]
        # Transform the pose to the map frame
        oTmG = lt.transform_pose(oTb, "map")

        # Open the gripper before picking up the object
        rospy.logwarn("Opening Gripper")
        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        # Move to the pre-grasp position and visualize the action
        rospy.logwarn("Picking up now")
        # BulletWorld.current_bullet_world.add_vis_axis(oTmG)
        # Execute Bool, because sometimes u only want to visualize the poses to pp.py things
        if execute:
            MoveTCPMotion(oTmG, self.arm, allow_gripper_collision=False).perform()

        # Calculate and apply any special knowledge offsets based on the robot and object type
        # Note: This currently includes robot-specific logic that should be generalized
        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))

        # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
        #  depending on robot
        if robot.name == "hsrb":
            if self.grasp == "top":
                if self.object_designator.type == "Metalbowl":
                    special_knowledge_offset.pose.position.y += 0.085
                    special_knowledge_offset.pose.position.x -= 0.03

        push_base = special_knowledge_offset
        # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
        #  depending on robot if we dont generlize this we will have a big list in the end of all robots
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
        # BulletWorld.current_bullet_world.add_vis_axis(special_knowledge_offsetTm)
        if execute:
            MoveTCPMotion(special_knowledge_offsetTm, self.arm, allow_gripper_collision=False).perform()

        rospy.logwarn("Pushing now")
        # BulletWorld.current_bullet_world.add_vis_axis(push_baseTm)
        if execute:
            MoveTCPMotion(push_baseTm, self.arm, allow_gripper_collision=False).perform()

        # Finalize the pick-up by closing the gripper and lifting the object
        rospy.logwarn("Close Gripper")
        MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm, allow_gripper_collision=True).perform()

        rospy.logwarn("Lifting now")
        liftingTm = push_baseTm
        liftingTm.pose.position.z += 0.03
        # BulletWorld.current_bullet_world.add_vis_axis(liftingTm)
        if execute:
            MoveTCPMotion(liftingTm, self.arm, allow_gripper_collision=False).perform()
        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        robot.attach(object, tool_frame)


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

    @with_tree
    def perform(self) -> None:
        lt = LocalTransformer()
        robot = World.robot
        execute = True
        # oTm = Object Pose in Frame map
        oTm = self.target_location

        if self.grasp == "top":
            oTm.pose.position.z += 0.05

        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = RobotDescription.current_robot_description.grasps[self.grasp]
        oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        # Set pose to the grasp rotation
        oTb.orientation = multiply_quaternions(oTb.orientation_as_list(), grasp_rotation)
        # Transform the pose to the map frame
        oTmG = lt.transform_pose(oTb, "map")

        rospy.logwarn("Placing now")
        #World.current_world.add_vis_axis(oTmG)
        if execute:
            MoveTCPMotion(oTmG, self.arm).perform()

        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
        if robot.name == "hsrb":
            z = 0.03
            if self.grasp == Grasp.TOP:
                z = 0.07
            push_base.pose.position.z += z
        # todo: make this for other robots
        push_baseTm = lt.transform_pose(push_base, "map")

        rospy.logwarn("Pushing now")
        #BulletWorld.current_bullet_world.add_vis_axis(push_baseTm)
        if execute:
            MoveTCPMotion(push_baseTm, self.arm).perform()

            #try:
            #    plan = MoveTCPMotion(side_push, self.arm) >> Monitor(monitor_func)
            #    plan.perform()
            #except (SensorMonitoringCondition):
            #    rospy.logwarn("Open Gripper")
            #    MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

        # Finalize the placing by opening the gripper and lifting the arm
        rospy.logwarn("Open Gripper")
        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        rospy.logwarn("Lifting now")
        liftingTm = push_baseTm
        liftingTm.pose.position.z += 0.08
        #BulletWorld.current_bullet_world.add_vis_axis(liftingTm)
        if execute:
            MoveTCPMotion(liftingTm, self.arm).perform()


@dataclass
class NavigateActionPerformable(ActionAbstract):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Location to which the robot should navigate
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
        # Tries to find a pick-up posotion for the robot that uses the given arm
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
    object_designator: ObjectDesignatorDescription.Object
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
        return DetectingMotion(object_type=self.object_designator.obj_type,
                               technique=self.technique, state=self.state).perform()


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

    def perform(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.object_designator.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, self.grasp).perform()

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
class PointingActionPerformable(ActionAbstract):
    x_coordinate: float
    """
    x coordinate where the robot points to (in map frame)
    """
    y_coordinate: float
    """
    y coordinate where the robot points to (in map frame)
    """
    z_coordinate: float
    """
    z coordinate where the robot points to (in map frame)
    """

    @with_tree
    def perform(self) -> None:
        PointingMotion(self.x_coordinate, self.y_coordinate, self.z_coordinate).perform()
