@init_giskard_interface
def allow_gripper_collision(gripper: Arms) -> None:
    """
    Allows the specified gripper to collide with anything.

    :param gripper: The gripper which can collide, either 'right', 'left' or 'all'
    """
    add_gripper_groups()
    giskard_wrapper.motion_goals.allow_collision("gripper")
    # for gripper_group in get_gripper_group_names():
    # if gripper in gripper_group or gripper == Arms.LEFT:
    #     giskard_wrapper.motion_goals.allow_collision(gripper_group, CollisionEntry.ALL)