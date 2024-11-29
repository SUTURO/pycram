from abc import ABC, abstractmethod
from dataclasses import dataclass

from geometry_msgs.msg import PointStamped

from pycram.datastructures.enums import *
from sqlalchemy.orm import Session
from .object_designator import ObjectDesignatorDescription, ObjectPart, RealObject
from ..designator import ResolutionError
from ..orm.base import ProcessMetaData
from ..failures import PerceptionObjectNotFound
from ..process_module import ProcessModuleManager
from ..robot_descriptions import robot_description
from ..orm.motion_designator import (MoveMotion as ORMMoveMotion, AccessingMotion as ORMAccessingMotion,
                                     MoveTCPMotion as ORMMoveTCPMotion, LookingMotion as ORMLookingMotion,
                                     MoveGripperMotion as ORMMoveGripperMotion, DetectingMotion as ORMDetectingMotion,
                                     OpeningMotion as ORMOpeningMotion, ClosingMotion as ORMClosingMotion,
                                     Motion as ORMMotionDesignator)
from ..datastructures.enums import ObjectType, Arms, GripperState, ExecutionType

from typing_extensions import Dict, Optional, get_type_hints
from ..datastructures.pose import Pose
from ..tasktree import with_tree
from ..designator import BaseMotion


@dataclass
class MoveMotion(BaseMotion):
    """
    Moves the robot to a designated location
    """

    target: Pose
    """
    Location to which the robot should be moved
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.navigate().execute(self)
        # return ProcessModule.perform(self)

    def to_sql(self) -> ORMMoveMotion:
        return ORMMoveMotion()

    def insert(self, session, *args, **kwargs) -> ORMMoveMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class MoveTCPMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: Pose
    """
    Target pose to which the TCP should be moved
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_tcp().execute(self)

    def to_sql(self) -> ORMMoveTCPMotion:
        return ORMMoveTCPMotion(self.arm, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveTCPMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class PickUpMotion(BaseMotion):
    """
    Lets the robot pick up a specific object
    """

    object_desig: ObjectDesignatorDescription.Object
    """
    Object designator describing the object to be picked up
    """
    arm: str
    """
    Arm that should be used for picking up the object
    """
    grasp: str
    """
    From which direction the object should be grasped, e.g. 'left', 'front', etc.
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.pick_up().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass

@dataclass
class PlaceMotion(BaseMotion):
    """
    Lets the robot place an object that was picked up
    """

    object: ObjectDesignatorDescription.Object
    """
    Object designator of the object to be placed
    """
    target: Pose
    """
    Pose at which the object should be placed
    """
    arm: str
    """
    Arm that is currently holding the object
    """
    grasp: str

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.place().execute(self)


@dataclass
class LookingMotion(BaseMotion):
    """
    Lets the robot look at a point
    """
    target: Pose

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.looking().execute(self)

    def to_sql(self) -> ORMLookingMotion:
        return ORMLookingMotion()

    def insert(self, session: Session, *args, **kwargs) -> ORMLookingMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: GripperState
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: Arms
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_gripper().execute(self)

    def to_sql(self) -> ORMMoveGripperMotion:
        return ORMMoveGripperMotion(self.motion, self.gripper, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveGripperMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


@dataclass
class DetectingMotion(BaseMotion):
    """
    Tries to detect an object in the FOV of the robot
    """
    technique: str
    """
    Technique means how the object should be detected, e.g. 'color', 'shape', 'all', etc. 
    """

    object_type: ObjectType
    """
    Type of the object that should be detected
    """

    state: Optional[str] = None
    """
    The state instructs our perception system to either start or stop the search for an object or human.
    Can also be used to describe the region or location where objects are perceived.
    """
    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        world_object = pm_manager.detecting().execute(self)

        if not world_object:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {self.object_type} in the FOV of the robot")
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            try:
                return RealObject.Object(world_object.name, world_object.obj_type,
                                        world_object, world_object.get_pose())
            except:
                return world_object


        return ObjectDesignatorDescription.Object(world_object.name, world_object.obj_type,
                                                  world_object)

    def to_sql(self) -> ORMDetectingMotion:
        return ORMDetectingMotion(self.object_type)

    def insert(self, session: Session, *args, **kwargs) -> ORMDetectingMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


@dataclass
class MoveArmJointsMotion(BaseMotion):
    """
    Moves the joints of each arm into the given position
    """

    left_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the left arm joints
    """
    right_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the right arm joints
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_arm_joints().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class WorldStateDetectingMotion(BaseMotion):
    """
    Detects an object based on the world state.
    """

    object_type: ObjectType
    """
    Object type that should be detected
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.world_state_detecting().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class MoveJointsMotion(BaseMotion):
    """
    Moves any joint on the robot
    """

    names: list
    """
    List of joint names that should be moved 
    """
    positions: list
    """
    Target positions of joints, should correspond to the list of names
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_joints().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.open().execute(self)

    def to_sql(self) -> ORMOpeningMotion:
        return ORMOpeningMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMOpeningMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion


@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.close().execute(self)

    def to_sql(self) -> ORMClosingMotion:
        return ORMClosingMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMClosingMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion

@dataclass
class TalkingMotion(BaseMotion):
    """
    Talking
    """
    cmd: str
    """
    Sentence what the robot should say
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.talk().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class PouringMotion(BaseMotion):
    """
    Designator for pouring
    """

    direction: str
    """
    The direction that should be used for pouring. For example, 'left' or 'right'
    """

    angle: float
    """
    the angle to move the gripper to
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.pour().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass



@dataclass
class HeadFollowMotion(BaseMotion):
    """
    continuously look at human
    """
    state: Optional[str]
    """
    Whether to start or stop motion

    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.head_follow().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class PointingMotion(BaseMotion):
    """
    Point at given Coordinates
    """
    goal_point: PointStamped
    """
    point the robot moves gripper to (in map frame)
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.pointing().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class DoorOpenMotion(BaseMotion):
    """
    Designator for opening a door
    """

    handle: str
    """
    name of the handle joint so that giskard knows how to open the door
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.door_opening().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass

@dataclass
class GraspHandleMotion(BaseMotion):
    """
    Designator for grasping a (door-)handle
    """

    handle: str
    """
    name of the handle joint so that giskard knows how to open the door
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.grasp_door_handle().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class GraspingDishwasherHandleMotion(BaseMotion):
    """
    Designator for grasping the dishwasher handle
    """

    handle_name: str
    """
    Name of the handle to grasp
    """
    arm: str
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.grasp_dishwasher_handle().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class HalfOpeningDishwasherMotion(BaseMotion):
    """
    Designator for half opening the dishwasher door to a given degree.
    """
    handle_name: str
    """
    Name of the dishwasher handle which is grasped
    """
    goal_state_half_open: float
    """
    Goal state of the door, defining the degree to open the door
    """
    arm: str
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.half_open_dishwasher().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class MoveArmAroundMotion(BaseMotion):
    """
    Designator for moving the arm around the dishwasher to further open the door.
    """

    handle_name: str
    """
    Name of the dishwasher handle which was grasped
    """
    arm: str
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_arm_around_dishwasher().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class FullOpeningDishwasherMotion(BaseMotion):
    """
    Designator for fully opening the dishwasher. Assumes that the door is already half opened and the arm is in the right position.
    """

    handle_name: str
    """
    Name of the dishwasher handle which was grasped
    """
    door_name: str
    """
    Name of the dishwasher door which should be opened
    """
    goal_state_full_open: float
    """
    Goal state of the door, defining the degree to open the door
    """
    arm: str
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.full_open_dishwasher().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass
