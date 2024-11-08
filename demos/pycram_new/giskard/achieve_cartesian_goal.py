@init_giskard_interface
@thread_safe
def achieve_cartesian_goal(goal_pose: Pose, tip_link: str, root_link: str) -> 'MoveResult':
    """
    Takes a cartesian position and tries to move the tip_link to this position using the chain defined by
    tip_link and root_link.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    :return: MoveResult message for this goal
    """

    root_link = 'map'
    tip_link = 'hand_gripper_tool_frame'
    sync_worlds()

    cart_monitor1 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
                                                                tip_link=tip_link,
                                                                goal_pose=_pose_to_pose_stamped(goal_pose),
                                                                position_threshold=0.02,
                                                                orientation_threshold=0.02)

    end_monitor = giskard_wrapper.monitors.add_local_minimum_reached(start_condition=cart_monitor1)

    giskard_wrapper.motion_goals.add_cartesian_pose(name='g1-cartesian',
                                                    root_link=root_link,
                                                    tip_link=tip_link,
                                                    goal_pose=_pose_to_pose_stamped(goal_pose),
                                                    end_condition=cart_monitor1)

    giskard_wrapper.monitors.add_end_motion(start_condition=end_monitor)
    # giskard_wrapper.motion_goals.avoid_all_collisions()
    # TODO: group 2 with CollisionEntry.ALL does not work -> not defined

    # giskard_wrapper.motion_goals.allow_collision(group1='gripper', group2='Milkpack')
    giskard_wrapper.motion_goals.avoid_collision(group1="hsrb", group2="kitchen")
    giskard_wrapper.motion_goals.avoid_collision(group1="gripper", group2="kitchen")

    return giskard_wrapper.execute()