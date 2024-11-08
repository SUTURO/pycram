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