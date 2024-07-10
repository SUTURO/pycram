from typing import List, Dict, Optional, TYPE_CHECKING

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Vector3Stamped, Vector3
from giskard_msgs.msg import CollisionEntry, WorldBody, Weights

from ..bullet_world import BulletWorld, Object
from ..pose import Pose
from ..robot_descriptions import robot_description
from ..utilities import tf_wrapper as tf

#
# if TYPE_CHECKING:
#     from giskardpy.python_interface.python_interface import GiskardWrapper
#     from giskard_msgs.msg import MoveResult, UpdateWorldResponse
giskard_wrapper = None
# giskard_wrapper: GiskardWrapper = None
giskard_update_service = None
is_init = False


def init_giskard_interface():
    global giskard_wrapper
    global giskard_update_service
    global is_init
    if is_init:
        return
    topics = list(map(lambda x: x[0], rospy.get_published_topics()))
    try:
        from giskardpy.python_interface.python_interface import GiskardWrapper
        from giskard_msgs.msg import WorldBody, MoveResult, CollisionEntry, Weights

        # from giskard_msgs.srv import UpdateWorldRequest, UpdateWorld, UpdateWorldResponse, RegisterGroupResponse

        if "/giskard/command/goal" in topics:
            giskard_wrapper = GiskardWrapper()
            # giskard_update_service = rospy.ServiceProxy("/giskard/update_world", UpdateWorld)
            is_init = True
            rospy.loginfo("Successfully initialized Giskard interface")
        else:
            rospy.logwarn("Giskard is not running, could not initialize Giskard interface")
    except ModuleNotFoundError as e:
        rospy.logwarn("Failed to import Giskard messages, giskard interface could not be initialized")


# Believe state management between pycram and giskard


def initial_adding_objects() -> None:
    """
    Adds object that are loaded in the BulletWorld to the Giskard belief state, if they are not present at the moment.
    """
    groups = giskard_wrapper.world.get_group_names()
    for obj in BulletWorld.current_bullet_world.objects:
        if obj != BulletWorld.robot and len(obj.links) >= 1:
            if obj.name != 'floor' or obj.type != "robot" or obj.name != "hsrb":
                name = obj.name + "_" + str(obj.id)

                if name not in groups:
                    spawn_object(obj)


def removing_of_objects() -> None:
    """
    Removes objects that are present in the Giskard belief state but not in the BulletWorld from the Giskard belief state.
    """
    groups = giskard_wrapper.world.get_group_names()
    if groups:
        object_names = list(
            map(lambda obj: obj.name + "_" + str(obj.id), BulletWorld.current_bullet_world.objects))
        diff = list(set(groups) - set(object_names))
        for grp in diff:
            giskard_wrapper.motion_goals.remove_group(grp)


def sync_worlds() -> None:
    """
    Synchronizes the BulletWorld and the Giskard belief state, this includes adding and removing objects to the Giskard
    belief state such that it matches the objects present in the BulletWorld and moving the robot to the position it is
    currently at in the BulletWorld.
    """
    init_giskard_interface()
    # add_gripper_groups()
    bullet_object_names = set()
    for obj in BulletWorld.current_bullet_world.objects:
        if obj.name != robot_description.name and len(obj.links) != 1:
            if obj.name != 'floor' or obj.type != "robot" or obj.name != "hsrb":
                bullet_object_names.add(obj.name + "_" + str(obj.id))

    giskard_object_names = set(giskard_wrapper.world.get_group_names())

    robot_name = {robot_description.name}
    if not bullet_object_names.union(robot_name).issubset(giskard_object_names):
        giskard_wrapper.world.clear()
    initial_adding_objects()


def clear() -> None:
    giskard_wrapper.world.clear()


def update_pose(object: Object) -> 'UpdateWorldResponse':
    """
    Sends an update message to giskard to update the object position. Might not work when working on the real robot just
    in standalone mode.

    :param object: Object that should be updated
    :return: An UpdateWorldResponse
    """
    return giskard_wrapper.motion_goals.update_group_pose(object.name + "_" + str(object.id), object.get_pose())


def spawn_object(object: Object) -> None:
    """
    Spawns a BulletWorld Object in the giskard belief state.

    :param object: BulletWorld object that should be spawned
    """
    if "hsrb" not in object.name:
        if hasattr(object, "path"):
            spawn_urdf(object.name + "_" + str(object.id), object.path, object.get_pose())
        else:
            geom = object.customGeom["size"]
            spawn_box(object.name + "_" + str(object.id), geom, object.get_pose())


def spawn_urdf(name: str, urdf_path: str, pose: Pose) -> 'UpdateWorldResponse':
    """
    Spawns an URDF in giskard's belief state.

    :param name: Name of the URDF
    :param urdf_path: Path to the URDF file
    :param pose: Pose in which the URDF should be spawned
    :return: An UpdateWorldResponse message
    """
    urdf_string = ""
    with open(urdf_path) as f:
        urdf_string = f.read()
    return giskard_wrapper.world.add_urdf(name, urdf_string, pose)


def spawn_mesh(name: str, path: str, pose: Pose) -> 'UpdateWorldResponse':
    """
    Spawns a mesh into giskard's belief state

    :param name: Name of the mesh
    :param path: Path to the mesh file
    :param pose: Pose in which the mesh should be spawned
    :return: An UpdateWorldResponse message
    """
    return giskard_wrapper.motion_goals.add_mesh(name, path, pose)


def spawn_box(name: str, size: tuple, pose: Pose) -> 'UpdateWorldResponse':
    """
    Spawns a mesh into giskard's belief state

    :param name: Name of the mesh
    :param path: Path to the mesh file
    :param pose: Pose in which the mesh should be spawned
    :return: An UpdateWorldResponse message
    """
    return giskard_wrapper.world.add_box(name, size, pose)


#
# # Sending Goals to Giskard
# def drive_brum(goal_poses: List[PoseStamped]) -> 'MoveResult':
#     """
#     Takes a list of poses geometry_msgs:pose_stamped and tries to move the robot to these positions.
#
#     :param goal_poses: List of poses that the robot should move to
#     :return: MoveResult message for this goal
#     """
#     root_link = 'map'
#     tip_link = 'base_link'
#     sync_worlds()
#     cart_monitor1 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
#                                                                 tip_link=tip_link,
#                                                                 goal_poses=goal_poses[-1],
#                                                                 name='cart goal 1')
#     giskard_wrapper.motion_goals.add_cartesian_poses(goal_poses=goal_poses,
#                                                     name='g1',
#                                                     root_link=root_link,
#                                                     tip_link=tip_link,
#                                                     end_condition=cart_monitor1)
#     giskard_wrapper.motion_goals.avoid_all_collisions()
#     return giskard_wrapper.execute()


def achieve_joint_goal(goal_poses: Dict[str, float]) -> 'MoveResult':
    """
    Takes a dictionary of joint position that should be achieved, the keys in the dictionary are the joint names and
    values are the goal joint positions.

    :param goal_poses: Dictionary with joint names and position goals
    :return: MoveResult message for this goal
    """
    # sync_worlds()

    giskard_wrapper.motion_goals.add_joint_position(goal_poses)
    giskard_wrapper.add_default_end_motion_conditions()
    giskard_wrapper.motion_goals.avoid_all_collisions()
    return giskard_wrapper.execute()


def achieve_placing_without_prepose(pose1, obj_desig, kitchen_obj):
    w = Weights()
    sync_worlds()
    obj_name = obj_desig.bullet_world_object.name + "_" + str(obj_desig.bullet_world_object.id)
    kitchen_name = kitchen_obj.name + "_" + str(kitchen_obj.id)
    pre_place_reached = giskard_wrapper.monitors.add_cartesian_pose(root_link='map',
                                                                    tip_link=obj_name,
                                                                    goal_pose=_pose_to_pose_stamped(pose1),
                                                                    position_threshold=0.06,
                                                                    name='pre placed reached')
    place_reached = giskard_wrapper.monitors.add_cartesian_pose(root_link='map',
                                                                tip_link=obj_name,
                                                                goal_pose=_pose_to_pose_stamped(pose1),
                                                                position_threshold=0.01,
                                                                name='placed reached')
    giskard_wrapper.motion_goals.add_cartesian_pose(_pose_to_pose_stamped(pose1), root_link='map', tip_link=obj_name,
                                                    weight=w.WEIGHT_BELOW_CA,
                                                    end_condition=place_reached)
    giskard_wrapper.motion_goals.avoid_all_collisions(end_condition=pre_place_reached)
    giskard_wrapper.motion_goals.allow_collision(group1=obj_name, group2=kitchen_name,
                                                 start_condition=pre_place_reached)
    giskard_wrapper.monitors.add_end_motion(start_condition=place_reached)
    return giskard_wrapper.execute()


def achieve_sequence_te(pose1, obj_desig):
    root_link = 'map'
    tip_link = "hand_gripper_tool_frame"
    # sync_worlds()
    cart_monitor1 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
                                                                tip_link=tip_link,
                                                                goal_pose=_pose_to_pose_stamped(pose1),
                                                                position_threshold=0.02,
                                                                orientation_threshold=0.04,
                                                                name='cart goal 1')
    giskard_wrapper.motion_goals.add_cartesian_pose(goal_pose=_pose_to_pose_stamped(pose1),
                                                    name='g1',
                                                    root_link=root_link,
                                                    tip_link=tip_link,
                                                    end_condition=cart_monitor1)
    giskard_wrapper.motion_goals.avoid_all_collisions()
    return giskard_wrapper.execute()


def achieve_sequence_pick_up(pose1):
    root_link = 'map'
    tip_link = 'hand_gripper_tool_frame'
    sync_worlds()
    cart_monitor1 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
                                                                tip_link=tip_link,
                                                                goal_pose=_pose_to_pose_stamped(pose1),
                                                                position_threshold=0.02,
                                                                orientation_threshold=0.02,
                                                                name='cart goal 1')
    # cart_monitor2 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
    #                                                             tip_link=tip_link,
    #                                                             goal_pose=_pose_to_pose_stamped(pose2),
    #                                                             position_threshold=0.03,
    #                                                             orientation_threshold=0.03,
    #                                                             name='cart goal 2',
    #                                                             start_condition=cart_monitor1)
    # gripper_closed = giskard_wrapper.monitors.add_close_hsr_gripper(start_condition=cart_monitor2,
    #                                                                 name='close gripper')
    # cart_monitor3 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
    #                                                             tip_link=tip_link,
    #                                                             goal_pose=_pose_to_pose_stamped(pose3),
    #                                                             name='cart goal 3',
    #                                                             start_condition=gripper_closed)
    # cart_monitor4 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
    #                                                             tip_link=tip_link,
    #                                                             goal_pose=_pose_to_pose_stamped(pose4),
    #                                                             name='cart goal 4',
    #                                                             start_condition=cart_monitor3)
    end_monitor = giskard_wrapper.monitors.add_local_minimum_reached(start_condition=cart_monitor1)

    giskard_wrapper.motion_goals.add_cartesian_pose(name='g1',
                                                    root_link=root_link,
                                                    tip_link=tip_link,
                                                    goal_pose=_pose_to_pose_stamped(pose1),
                                                    end_condition=cart_monitor1)
    # giskard_wrapper.motion_goals.add_cartesian_pose(name='g2',
    #                                                 root_link=root_link,
    #                                                 tip_link=tip_link,
    #                                                 goal_pose=_pose_to_pose_stamped(pose2),
    #                                                 start_condition=cart_monitor1,
    #                                                 end_condition=cart_monitor2)

    # giskard_wrapper.motion_goals.add_cartesian_pose(name='g3',
    #                                                 root_link=root_link,
    #                                                 tip_link=tip_link,
    #                                                 goal_pose=_pose_to_pose_stamped(pose3),
    #                                                 start_condition=gripper_closed,
    #                                                 end_condition=cart_monitor3)
    # giskard_wrapper.motion_goals.add_cartesian_pose(name='g4',
    #                                                 root_link=root_link,
    #                                                 tip_link=tip_link,
    #                                                 goal_pose=_pose_to_pose_stamped(pose4),
    #                                                 start_condition=cart_monitor3,
    #                                                 end_condition=cart_monitor4)

    giskard_wrapper.monitors.add_end_motion(start_condition=end_monitor)
    #giskard_wrapper.motion_goals.avoid_all_collisions()
    #giskard_wrapper.motion_goals.allow_collision(group1='gripper', group2=CollisionEntry.ALL)
    giskard_wrapper.motion_goals.allow_all_collisions()
    return giskard_wrapper.execute()

def test(config):
    giskard_wrapper.motion_goals.add_joint_position(config)
    # js_reached = giskard.monitors.add_joint_position(js1, threshold=0.03)
    # giskard.monitors.add_end_motion(start_condition=js_reached)
    giskard_wrapper.motion_goals.allow_all_collisions()

    giskard_wrapper.add_default_end_motion_conditions()
    giskard_wrapper.execute()

def achieve_attached(obj_desig, tip_link='hand_gripper_tool_frame'):
    root_link = 'map'
    sync_worlds()
    giskard_wrapper.world.update_parent_link_of_group(
        name=obj_desig.name + "_" + str(obj_desig.id), parent_link=tip_link)

def achieve_sequence_place(pose1, pose2):
    root_link = 'map'
    tip_link = 'hand_gripper_tool_frame'
    cart_monitor1 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
                                                                tip_link=tip_link,
                                                                goal_pose=_pose_to_pose_stamped(pose1),
                                                                position_threshold=0.02,
                                                                orientation_threshold=0.04,
                                                                name='cart goal 1')
    cart_monitor2 = giskard_wrapper.monitors.add_cartesian_pose(root_link=root_link,
                                                                tip_link=tip_link,
                                                                goal_pose=_pose_to_pose_stamped(pose2),
                                                                name='cart goal 2',
                                                                start_condition=cart_monitor1)

    end_monitor = giskard_wrapper.monitors.add_local_minimum_reached(start_condition=cart_monitor2)

    giskard_wrapper.motion_goals.add_cartesian_pose(name='g1',
                                                    root_link=root_link,
                                                    tip_link=tip_link,
                                                    goal_pose=_pose_to_pose_stamped(pose1),
                                                    end_condition=cart_monitor1)
    giskard_wrapper.motion_goals.add_cartesian_pose(name='g2',
                                                    root_link=root_link,
                                                    tip_link=tip_link,
                                                    goal_pose=_pose_to_pose_stamped(pose2),
                                                    start_condition=cart_monitor1,
                                                    end_condition=cart_monitor2)
    giskard_wrapper.motion_goals.avoid_all_collisions()
    giskard_wrapper.monitors.add_end_motion(start_condition=end_monitor)
    return giskard_wrapper.execute()


def cml(drive_back):
    try:
        print("in cml")
        giskard_wrapper.motion_goals.add_carry_my_luggage(name='cmb', drive_back=drive_back)
        giskard_exe= giskard_wrapper.execute()
        print(giskard_exe)
    except:
        if giskard_exe.error.code == 2:
            print("works fine")
        else:
            print("cml error")



def achieve_cartesian_goal(goal_pose: Pose, tip_link: str, root_link: str) -> 'MoveResult':
    """
    Takes a cartesian position and tries to move the tip_link to this position using the chain defined by
    tip_link and root_link.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.motion_goals.avoid_all_collisions()
    giskard_wrapper.motion_goals.add_cartesian_pose(_pose_to_pose_stamped(goal_pose), tip_link, root_link)
    return giskard_wrapper.execute()


def achieve_straight_cartesian_goal(goal_pose: Pose, tip_link: str,
                                    root_link: str) -> 'MoveResult':
    """
    Takes a cartesian position and tries to move the tip_link to this position in a straight line, using the chain
    defined by tip_link and root_link.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.motion_goals.set_straight_cart_goal(_pose_to_pose_stamped(goal_pose), tip_link, root_link)
    return giskard_wrapper.execute()


def achieve_translation_goal(goal_point: List[float], tip_link: str, root_link: str) -> 'MoveResult':
    """
    Tries to move the tip_link to the position defined by goal_point using the chain defined by root_link and
    tip_link. Since goal_point only defines the position but no rotation, rotation is not taken into account.

    :param goal_point: The goal position of the tip_link
    :param tip_link: The link which should be moved to goal_point as well as the end of the used chain
    :param root_link: The start link of the chain
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.motion_goals.set_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
    return giskard_wrapper.execute()


def achieve_straight_translation_goal(goal_point: List[float], tip_link: str, root_link: str) -> 'MoveResult':
    """
    Tries to move the tip_link to the position defined by goal_point in a straight line, using the chain defined by
    root_link and tip_link. Since goal_point only defines the position but no rotation, rotation is not taken into account.

    :param goal_point: The goal position of the tip_link
    :param tip_link: The link which should be moved to goal_point as well as the end of the used chain
    :param root_link: The start link of the chain
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.motion_goals.set_straight_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
    return giskard_wrapper.execute()


def achieve_rotation_goal(quat: List[float], tip_link: str, root_link: str) -> 'MoveResult':
    """
    Tries to bring the tip link into the rotation defined by quat using the chain defined by root_link and
    tip_link.

    :param quat: The rotation that should be achieved, given as a quaternion
    :param tip_link: The link that should be in the rotation defined by quat
    :param root_link: The start link of the chain
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.motion_goals.set_rotation_goal(make_quaternion_stamped(quat), tip_link, root_link)
    return giskard_wrapper.execute()


def achieve_align_planes_goal(goal_normal: List[float], tip_link: str, tip_normal: List[float],
                              root_link: str) -> 'MoveResult':
    """
    Tries to align the plane defined by tip normal with goal_normal using the chain between root_link and
    tip_link.

    :param goal_normal: The goal plane, given as a list of XYZ
    :param tip_link: The end link of the chain that should be used.
    :param tip_normal: The plane that should be aligned with goal_normal, given as a list of XYZ
    :param root_link: The starting link of the chain that should be used.
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.motion_goals.set_align_planes_goal(make_vector_stamped(goal_normal), tip_link,
                                                       make_vector_stamped(tip_normal),
                                                       root_link)
    return giskard_wrapper.execute()


def achieve_open_container_goal(tip_link: str, environment_link: str, goal_state: Optional[float] = None,
                                special_door: Optional[bool] = False) -> 'MoveResult':
    """
    Tries to open a container in an environment, this only works if the container was added as a URDF. This goal assumes
    that the handle was already grasped. Can only handle container with 1 DOF

    :param tip_link: The End effector that should open the container.
    :param environment_link: The name of the handle for this container.
    :param goal_state: The degree to which the door should be opened.
    :return: MoveResult message for this goal.
    """
    print(tip_link)
    print(environment_link)
    if goal_state is None:
        giskard_wrapper.motion_goals.set_open_container_goal(tip_link, environment_link)
    else:
        giskard_wrapper.motion_goals.set_open_container_goal(tip_link, environment_link, goal_joint_state=goal_state,
                                                             special_door=special_door)

    giskard_wrapper.motion_goals.allow_all_collisions()
    return giskard_wrapper.execute()


def set_hsrb_dishwasher_door_around(handle_name: str) -> 'MoveResult':
    """
    Moves the arm around the dishwasher door after the first opening action. Dishwasher is in this state half open.

    :param handle_name: the name of the handle the HSR was grasping.
    :return: MoveResult message for this goal
    """
    giskard_wrapper.motion_goals.set_hsrb_dishwasher_door_around(handle_name)
    giskard_wrapper.execute()


def fully_open_dishwasher_door(handle_name: str, door_name: str) -> 'MoveResult':
    """
    After the first opening part, the dishwasher is half open.
    Movement to move the arm around the dishwasher and bringing the arm in a position to push the door down.

    :param handle_name: The name of the handle of the container that was half opened.
    :param door_name: The name of the container door, where the arm needs to be moved around and aligned.
    :return: MoveResult message for this goal.
    """

    giskard_wrapper.motion_goals.set_hsrb_align_to_push_door_goal(handle_name, door_name)
    giskard_wrapper.execute()

    giskard_wrapper.motion_goals.set_hsrb_pre_push_door_goal(handle_name=handle_name, hinge_frame_id=door_name)
    giskard_wrapper.motion_goals.allow_all_collisions()
    giskard_wrapper.execute()


def achieve_close_container_goal(tip_link: str, environment_link: str) -> 'MoveResult':
    """
    Tries to close a container, this only works if the container was added as a URDF. Assumes that the handle of the
    container was already grasped. Can only handle container with 1 DOF.

    :param tip_link: Link name that should be used to close the container.
    :param environment_link: Name of the handle
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.motion_goals.set_close_container_goal(tip_link, environment_link)
    return giskard_wrapper.execute()


def achieve_tilting_goal(direction: str, angle: float):
    """
    tilts the gripper to the given angle
    :param direction: The direction that should be used for pouring. For example, 'left' or 'right'.
    :param angle: The angle that the robot tilts his gripper to
    :return: MoveResult message for this goal
    """
    rospy.loginfo("pouring")
    # sync_worlds()
    giskard_wrapper.motion_goals.tilting(direction, angle)
    return giskard_wrapper.execute()


# Managing collisions
def achieve_gripper_motion_goal(motion: str):
    """
    Opens or closes the gripper
    """
    rospy.loginfo("giskard change_gripper_state: " + motion)
    giskard_wrapper.motion_goals.change_gripper_state(motion)

    # return giskard_wrapper.execute()


def allow_gripper_collision(gripper: str):
    """
    Allows the specified gripper to collide with anything.

    :param gripper: The gripper which can collide, either 'right', 'left' or 'both'
    :return:
    """
    add_gripper_groups()
    if gripper == "right":
        giskard_wrapper.motion_goals.allow_collision("right_gripper", CollisionEntry.ALL)
    elif gripper == "left":
        giskard_wrapper.motion_goals.allow_collision("left_gripper", CollisionEntry.ALL)
    elif gripper == "both":
        giskard_wrapper.motion_goals.allow_collision("right_gripper", CollisionEntry.ALL)
        giskard_wrapper.motion_goals.allow_collision("left_gripper", CollisionEntry.ALL)


# todo niemand denkt an hsr :;(
def add_gripper_groups() -> None:
    """
    Adds the gripper links as a group for collision avoidance.

    :return: Response of the RegisterGroup Service
    """
    if "left_gripper" not in giskard_wrapper.motion_goals.get_group_names():
        for gripper in ["left"]:
            root_link = robot_description.chains[gripper].gripper.links[-1]
            giskard_wrapper.motion_goals.register_group(gripper + "_gripper", root_link, robot_description.name)


def avoid_all_collisions() -> None:
    """
    Will avoid all collision for the next goal.
    """
    giskard_wrapper.motion_goals.avoid_all_collisions()


def allow_self_collision() -> None:
    """
    Will allow the robot collision with itself.
    """
    giskard_wrapper.motion_goals.allow_self_collision()


def avoid_collisions(object1: Object, object2: Object) -> None:
    """
    Will avoid collision between the two objects for the next goal.

    :param object1: The first BulletWorld Object
    :param object2: The second BulletWorld Object
    """
    giskard_wrapper.motion_goals.avoid_collision(-1, object1.name + "_" + str(object1.id),
                                                 object2.name + "_" + str(object2.id))


def _pose_to_pose_stamped(pose: Pose) -> PoseStamped:
    """
    Transforms a PyCRAM pose to a PoseStamped message, this is necessary since Giskard NEEDS a PoseStamped message
    otherwise it will crash.

    :param pose: PyCRAM pose that should be converted
    :return: An equivalent PoseStamped message
    """
    ps = PoseStamped()
    ps.pose = pose.pose
    ps.header = pose.header

    return ps


def move_arm_to_point(point: PointStamped):
    """
    moves arm to given position
    :param point: point
    """
    print("in move arm")
    p_axis = Vector3Stamped()
    p_axis.header.frame_id = "hand_gripper_tool_frame"
    p_axis.vector.x = 0
    p_axis.vector.y = 0
    p_axis.vector.z = 1
    giskard_wrapper.motion_goals.add_pointing(goal_point=point,
                                      tip_link="hand_gripper_tool_frame",
                                      pointing_axis=p_axis,
                                      root_link="map")
    giskard_wrapper.add_default_end_motion_conditions()
    giskard_wrapper.execute()

def move_head_to_human():
    """
    continously moves head in direction of perceived human
    """

    giskard_wrapper.motion_goals.continuous_pointing_head()
    return giskard_wrapper.execute(wait=False)


# def stop_looking():
#     """
#     stops the move_head_to_human function so that hsr looks forward
#     """
#
#     # cancels all goals in giskard
#     # giskard_wrapper.cancel_all_goals()
#     # moves hsr in standard position
#     giskard_wrapper.take_pose("park")
#     giskard_wrapper.execute(wait=False)
#     rospy.loginfo("hsr looks forward instead of looking at human")


def cancel_all_called_goals():
    giskard_wrapper.cancel_all_goals()
    rospy.loginfo("Canceling all goals towards Giskard")


def _pose_to_pose_stamped(pose: Pose) -> PoseStamped:
    """
    Transforms a PyCRAM pose to a PoseStamped message, this is necessary since Giskard NEEDS a PoseStamped message
    otherwise it will crash.

    :param pose: PyCRAM pose that should be converted
    :return: An equivalent PoseStamped message
    """
    ps = PoseStamped()
    ps.pose = pose.pose
    ps.header = pose.header

    return ps


def move_head_to_pose(pose1: PoseStamped):
    """
    moves head to given position
    :param pose: pose that head will rotate to
    """
    print("we are in new giskard")
    p_axis = Vector3Stamped()
    p_axis.header.frame_id = 'head_center_camera_frame'
    p_axis.vector.x = 0
    p_axis.vector.y = 0
    p_axis.vector.z = 1

    pointSt = PointStamped()
    pointSt.header = pose1.header
    pointSt.point = pose1.pose.position
    # sync_worlds()
    print(pointSt)
    giskard_wrapper.motion_goals.add_pointing(goal_point=pointSt,
                                              name='g1',
                                              root_link="base_footprint",
                                              tip_link="head_center_camera_frame",
                                              pointing_axis=p_axis)
    # giskard_wrapper.motion_goals.allow_all_collisions()
    giskard_wrapper.add_default_end_motion_conditions()
    giskard_wrapper.execute()


def allow_all_collisions():
    giskard_wrapper.motion_goals.allow_all_collisions()


def spawn_kitchen():
    env_urdf = rospy.get_param('/iai_kitchen')
    kitchen_pose = tf.lookup_pose('map', 'iai_kitchen/urdf_main')
    print(kitchen_pose)
    giskard_wrapper.add_urdf(name='arena',
                             urdf=env_urdf,
                             pose=kitchen_pose)


#
def grasp_doorhandle(handle_name: str):
    print("grasp handle")

    giskard_wrapper.motion_goals.hsrb_door_handle_grasp(handle_name=handle_name)
    giskard_wrapper.motion_goals.allow_all_collisions()
    giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()
#
#
# def grasp_handle(handle_name: str):
#     """
#     grasps the dishwasher handle.
#
#     :param handle_name: name of the dishwasher handle, which should be grasped
#     """
#     giskard_wrapper.set_hsrb_dishwasher_door_handle_grasp(handle_name, grasp_bar_offset=0.035)
#     giskard_wrapper.execute()
#
#
def open_doorhandle(handle_name: str):
    giskard_wrapper.motion_goals.hsrb_open_door_goal(door_handle_link=handle_name)
    giskard_wrapper.motion_goals.allow_all_collisions()
    #giskard_wrapper.add_default_end_motion_conditions()
    return giskard_wrapper.execute()