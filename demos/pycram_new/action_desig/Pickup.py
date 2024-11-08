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