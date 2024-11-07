@dataclass
class DetectingMotion(BaseMotion):
    """
    Tries to detect an object in the FOV of the robot
    """

    object_type: ObjectType
    """
    Type of the object that should be detected
    """
    technique: str
    """
    Technique means how the object should be detected, e.g. 'color', 'shape', 'region', etc. 
    Or 'all' if all objects should be detected
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
        if ProcessModuleManager.execution_type == "real":
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