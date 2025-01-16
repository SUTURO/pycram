from __future__ import annotations

import dataclasses
from typing_extensions import List, Optional, Callable, TYPE_CHECKING
import sqlalchemy.orm
from ..datastructures.enums import ObjectType
from ..datastructures.world import World
from ..world_concepts.world_object import Object as WorldObject
from ..designator import ObjectDesignatorDescription
from ..orm.base import ProcessMetaData
from ..orm.object_designator import (BelieveObject as ORMBelieveObject, ObjectPart as ORMObjectPart)
from ..datastructures.pose import Pose
from ..external_interfaces.robokudo import *
from std_msgs.msg import String
from ..external_interfaces.robokudo import send_query

if TYPE_CHECKING:
    import owlready2


class BelieveObject(ObjectDesignatorDescription):
    """
    Description for Objects that are only believed in.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        """
        Concrete object that is believed in.
        """

        def to_sql(self) -> ORMBelieveObject:
            return ORMBelieveObject(name=self.name, obj_type=self.obj_type)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMBelieveObject:
            metadata = ProcessMetaData().insert(session)
            self_ = self.to_sql()
            self_.process_metadata = metadata
            session.add(self_)

            return self_


class ObjectPart(ObjectDesignatorDescription):
    """
    Object Designator Descriptions for Objects that are part of some other object.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):

        # The rest of attributes is inherited
        part_pose: Pose

        def to_sql(self) -> ORMObjectPart:
            return ORMObjectPart(obj_type=self.obj_type, name=self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectPart:
            metadata = ProcessMetaData().insert(session)
            pose = self.part_pose.insert(session)
            obj = self.to_sql()
            obj.process_metadata = metadata
            obj.pose = pose
            session.add(obj)

            return obj

    def __init__(self, names: List[str],
                 part_of: ObjectDesignatorDescription.Object,
                 type: Optional[ObjectType] = None,
                 resolver: Optional[Callable] = None):
        """
        Describing the relationship between an object and a specific part of it.

        :param names: Possible names for the part
        :param part_of: Parent object of which the part should be described
        :param type: Type of the part
        :param resolver: An alternative specialized_designators to resolve the input parameter to an object designator
        :param ontology_concept_holders: A list of ontology concepts that the object part is categorized as or associated with
        """
        super().__init__(names, type, resolver)

        if not part_of:
            raise AttributeError("part_of cannot be None.")

        self.type: Optional[ObjectType] = type
        self.names: Optional[List[str]] = names
        self.part_of = part_of

    def ground(self) -> Object:
        """
        Default specialized_designators, returns the first result of the iterator of this instance.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self):
        """
        Iterates through every possible solution for the given input parameter.

        :yield: A resolved Object designator
        """
        for name in self.names:
            if name in self.part_of.world_object.link_name_to_id.keys():
                yield self.Object(name, self.type, self.part_of.world_object,
                                  self.part_of.world_object.get_link_pose(name))


class LocatedObject(ObjectDesignatorDescription):
    """
    Description for KnowRob located objects.
    **Currently has no specialized_designators**
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        reference_frame: str
        """
        Reference frame in which the position is given
        """
        timestamp: float
        """
        Timestamp at which the position was valid
        """

    def __init__(self, names: List[str], types: List[str],
                 reference_frames: List[str], timestamps: List[float], resolver: Optional[Callable] = None,
                 ontology_concept_holders: Optional[List[owlready2.Thing]] = None):
        """
        Describing an object resolved through knowrob.

        :param names: List of possible names describing the object
        :param types: List of possible types describing the object
        :param reference_frames: Frame of reference in which the object position should be
        :param timestamps: Timestamps for which positions should be returned
        :param resolver: An alternative specialized_designators that resolves the input parameter to an object designator.
        :param ontology_concept_holders: A list of ontology concepts that the object is categorized as
        """
        super(LocatedObject, self).__init__(names, types, resolver, ontology_concept_holders)
        self.reference_frames: List[str] = reference_frames
        self.timestamps: List[float] = timestamps


@DeprecationWarning
# Depricated class, this will be done differently
class RealObject(ObjectDesignatorDescription):
    """
    Object designator representing an object in the real world, when resolving this object designator description ]
    RoboKudo is queried to perceive an object fitting the given criteria. Afterward the specialized_designators tries to match
    the found object to an Object in the World.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        pose: Pose
        """
        Pose of the perceived object
        """

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[str]] = None,
                 world_object: WorldObject = None, resolver: Optional[Callable] = None):
        """

        :param names:
        :param types:
        :param world_object:
        :param resolver:
        """
        super().__init__(resolver)
        self.types: Optional[List[str]] = types
        self.names: Optional[List[str]] = names
        self.world_object: WorldObject = world_object

    def __iter__(self):
        """
        Queries RoboKudo for objects that fit the description and then iterates over all World objects that have
        the same type to match a World object to the real object.

        :yield: A resolved object designator with reference world object
        """
        object_candidates = query_object(self)
        for obj_desig in object_candidates:
            for world_obj in World.get_object_by_type(obj_desig.obj_type):
                obj_desig.world_object = world_obj
                yield obj_desig
                # if bullet_obj.get_pose().dist(obj_deisg.pose) < 0.05:
                #     obj_deisg.bullet_world_object = bullet_obj
                #     yield obj_deisg


class HumanDescription:
    """
    Class that represents humans. this class does not spawn a human in a simulation.
    """

    def __init__(self, name: String, fav_drink: Optional = None,
                 pose: Optional = None, attributes: Optional = None, interests: Optional[List] = None):
        """
        :param name: name of human
        :param fav_drink: favorite drink of human
        :param pose: last known pose of human
        """

        # TODO: coordinate with Perception on what is easy to implement
        # characteristics to consider: height, hair color, and age.
        # self.human_pose = Fluent()
        self.name = name
        self.fav_drink = fav_drink
        self.pose = pose
        self.attributes = attributes
        if interests is None:
            self.interests = []
        else:
            self.interests = interests
        self.id = -1

    def set_id(self, new_id: int):
        """
        function for changing id of human
        is given by perception with face recognition
        :param new_id: new id of human
        """
        self.id = new_id

    def add_interests(self, new_interest: str):
        """
        function for adding interests of a human
        :param new_interest: new hobby of human
        """
        self.interests.append(new_interest)

    def remove_interests(self, index_to_remove: int):
        """
        function for removing items from interests-list
        :param index_to_remove: index of item that gets removed
        """

        if len(self.interests) > index_to_remove:
            self.interests.pop(index_to_remove)
        else:
            rospy.logerr("nothing removed, index to high")

    def set_name(self, new_name):
        """
        function for changing name of human
        :param new_name: new name of human
        """
        self.name = new_name

    def set_drink(self, new_drink):
        """
        function for changing/setting favorite drink of human
        :param new_drink: name of drink
        """
        self.fav_drink = new_drink

    def set_pose(self, new_pose):
        """
        function for changing pose of human
        :param new_pose: new pose of human
        """
        print("in set pose")
        self.pose = new_pose

    def set_attributes(self, attribute_list):
        """
        function for setting attributes
        :param attribute_list: list with attributes: gender, headgear, kind of clothes, bright/dark clothes
        """
        self.attributes = attribute_list


class CustomerDescription(HumanDescription):
    def __init__(self, id: int, pose: Pose, order: Optional = None):
        self.id = id
        self.pose = pose
        self.order = order

    def get_pose(self):
        return self.pose

    def set_pose(self, new_pose):
        self.pose = new_pose
        print(self.pose)
    def get_id(self):
        return id
    def set_id(self, new_id):
        self.id = new_id

    def set_order(self, order : List[(String, int)]):
        self.order = order

