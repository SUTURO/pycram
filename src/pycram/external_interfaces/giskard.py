from typing import List, Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Vector3Stamped
from giskard_msgs.msg import CollisionEntry, WorldBody

from ..bullet_world import BulletWorld, Object
from ..pose import Pose
from ..robot_descriptions import robot_description
from ..utilities import tf_wrapper as tf

giskard_wrapper = None
giskard_update_service = None
is_init = False
giskard_error_state = None

def init_giskard_interface():
    global giskard_wrapper
    global giskard_update_service
    global is_init
    if is_init:
        return
    topics = list(map(lambda x: x[0], rospy.get_published_topics()))
    try:
        from giskardpy.python_interface.old_python_interface import OldGiskardWrapper
        from giskard_msgs.msg import WorldBody, MoveResult, CollisionEntry
        # from giskard_msgs.srv import UpdateWorldRequest, UpdateWorld, UpdateWorldResponse, RegisterGroupResponse

        if "/giskard/command/goal" in topics:
            giskard_wrapper = OldGiskardWrapper()
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
    groups = giskard_wrapper.get_group_names()
    for obj in BulletWorld.current_bullet_world.objects:
        if obj != BulletWorld.robot and len(obj.links) >= 1:
            if obj.name != 'floor':
                name = obj.name + "_" + str(obj.id)

                if name not in groups:
                    spawn_object(obj)


def removing_of_objects() -> None:
    """
    Removes objects that are present in the Giskard belief state but not in the BulletWorld from the Giskard belief state.
    """
    groups = giskard_wrapper.get_group_names()
    if groups:
        object_names = list(
            map(lambda obj: obj.name + "_" + str(obj.id), BulletWorld.current_bullet_world.objects))
        diff = list(set(groups) - set(object_names))
        for grp in diff:
            giskard_wrapper.remove_group(grp)


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
            if obj.name != 'floor':
                bullet_object_names.add(obj.name + "_" + str(obj.id))

    giskard_object_names = set(giskard_wrapper.get_group_names())
    robot_name = {robot_description.name}
    if not bullet_object_names.union(robot_name).issubset(giskard_object_names):
        giskard_wrapper.clear_world()
    initial_adding_objects()


def update_pose(object: Object) -> 'UpdateWorldResponse':
    """
    Sends an update message to giskard to update the object position. Might not work when working on the real robot just
    in standalone mode.

    :param object: Object that should be updated
    :return: An UpdateWorldResponse
    """
    return giskard_wrapper.update_group_pose(object.name + "_" + str(object.id), object.get_pose())


def spawn_object(object: Object) -> None:
    """
    Spawns a BulletWorld Object in the giskard belief state.

    :param object: BulletWorld object that should be spawned
    """
    if hasattr(object, "path"):
        spawn_urdf(object.name + "_" + str(object.id), object.path, object.get_pose())
    else:
        geom = object.customGeom["size"]
        spawn_box(object.name + "_" + str(object.id), geom, object.get_pose())


def remove_object(object: Object) -> 'UpdateWorldResponse':
    """
    Removes an object from the giskard belief state.

    :param object: The BulletWorld Object that should be removed
    """
    return giskard_wrapper.remove_group(object.name + "_" + str(object.id))


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
    return giskard_wrapper.add_urdf(name, urdf_string, pose)


def spawn_mesh(name: str, path: str, pose: Pose) -> 'UpdateWorldResponse':
    """
    Spawns a mesh into giskard's belief state

    :param name: Name of the mesh
    :param path: Path to the mesh file
    :param pose: Pose in which the mesh should be spawned
    :return: An UpdateWorldResponse message
    """
    return giskard_wrapper.add_mesh(name, path, pose)


def spawn_box(name: str, size: tuple, pose: Pose) -> 'UpdateWorldResponse':
    """
    Spawns a mesh into giskard's belief state

    :param name: Name of the mesh
    :param path: Path to the mesh file
    :param pose: Pose in which the mesh should be spawned
    :return: An UpdateWorldResponse message
    """
    return giskard_wrapper.add_box(name, size, pose)


# Sending Goals to Giskard


def achieve_joint_goal(goal_poses: Dict[str, float]) -> 'MoveResult':
    """
    Takes a dictionary of joint position that should be achieved, the keys in the dictionary are the joint names and
    values are the goal joint positions.

    :param goal_poses: Dictionary with joint names and position goals
    :return: MoveResult message for this goal
    """
    sync_worlds()
    giskard_wrapper.set_joint_goal(goal_poses)
    return giskard_wrapper.execute()


def achieve_cartesian_goal(goal_pose: Pose, tip_link: str, root_link: str) -> 'MoveResult':
    """
    Takes a cartesian position and tries to move the tip_link to this position using the chain defined by
    tip_link and root_link.

    :param goal_pose: The position which should be achieved with tip_link
    :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
    :param root_link: The starting link of the chain which should be used to achieve this goal
    :return: MoveResult message for this goal
    """
    #sync_worlds()
    giskard_wrapper.avoid_all_collisions()
    giskard_wrapper.set_cart_goal(_pose_to_pose_stamped(goal_pose), tip_link, root_link)
    giskard_error_state = giskard_wrapper.execute()
    return giskard_error_state


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
    giskard_wrapper.set_straight_cart_goal(_pose_to_pose_stamped(goal_pose), tip_link, root_link)
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
    giskard_wrapper.set_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
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
    giskard_wrapper.set_straight_translation_goal(make_point_stamped(goal_point), tip_link, root_link)
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
    giskard_wrapper.set_rotation_goal(make_quaternion_stamped(quat), tip_link, root_link)
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
    giskard_wrapper.set_align_planes_goal(make_vector_stamped(goal_normal), tip_link, make_vector_stamped(tip_normal),
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
        giskard_wrapper.set_open_container_goal(tip_link, environment_link)
    else:
        giskard_wrapper.set_open_container_goal(tip_link, environment_link, goal_joint_state=goal_state,
                                                special_door=special_door)

    giskard_wrapper.allow_all_collisions()
    return giskard_wrapper.execute()


def set_hsrb_dishwasher_door_around(handle_name: str) -> 'MoveResult':
    """
    Moves the arm around the dishwasher door after the first opening action. Dishwasher is in this state half open.

    :param handle_name: the name of the handle the HSR was grasping.
    :return: MoveResult message for this goal
    """
    giskard_wrapper.set_hsrb_dishwasher_door_around(handle_name)
    giskard_wrapper.execute()


def fully_open_dishwasher_door(handle_name: str, door_name: str) -> 'MoveResult':
    """
    After the first opening part, the dishwasher is half open.
    Movement to move the arm around the dishwasher and bringing the arm in a position to push the door down.

    :param handle_name: The name of the handle of the container that was half opened.
    :param door_name: The name of the container door, where the arm needs to be moved around and aligned.
    :return: MoveResult message for this goal.
    """

    giskard_wrapper.set_hsrb_align_to_push_door_goal(handle_name, door_name)
    giskard_wrapper.execute()

    giskard_wrapper.set_hsrb_pre_push_door_goal(handle_name=handle_name, hinge_frame_id=door_name)
    giskard_wrapper.allow_all_collisions()
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
    giskard_wrapper.set_close_container_goal(tip_link, environment_link)
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
    giskard_wrapper.tilting(direction, angle)
    return giskard_wrapper.execute()


# Managing collisions
def achieve_gripper_motion_goal(motion: str):
    """
    Opens or closes the gripper
    """
    rospy.loginfo("giskard change_gripper_state: " + motion)
    giskard_wrapper.change_gripper_state(motion)

    # return giskard_wrapper.execute()


def allow_gripper_collision(gripper: str):
    """
    Allows the specified gripper to collide with anything.

    :param gripper: The gripper which can collide, either 'right', 'left' or 'both'
    :return:
    """
    add_gripper_groups()
    if gripper == "right":
        giskard_wrapper.allow_collision("right_gripper", CollisionEntry.ALL)
    elif gripper == "left":
        giskard_wrapper.allow_collision("left_gripper", CollisionEntry.ALL)
    elif gripper == "both":
        giskard_wrapper.allow_collision("right_gripper", CollisionEntry.ALL)
        giskard_wrapper.allow_collision("left_gripper", CollisionEntry.ALL)


# todo niemand denkt an hsr :;(
def add_gripper_groups() -> None:
    """
    Adds the gripper links as a group for collision avoidance.

    :return: Response of the RegisterGroup Service
    """
    if "left_gripper" not in giskard_wrapper.get_group_names():
        for gripper in ["left"]:
            root_link = robot_description.chains[gripper].gripper.links[-1]
            giskard_wrapper.register_group(gripper + "_gripper", root_link, robot_description.name)


def avoid_all_collisions() -> None:
    """
    Will avoid all collision for the next goal.
    """
    giskard_wrapper.avoid_all_collisions()


def allow_self_collision() -> None:
    """
    Will allow the robot collision with itself.
    """
    giskard_wrapper.allow_self_collision()


def avoid_collisions(object1: Object, object2: Object) -> None:
    """
    Will avoid collision between the two objects for the next goal.

    :param object1: The first BulletWorld Object
    :param object2: The second BulletWorld Object
    """
    giskard_wrapper.avoid_collision(-1, object1.name + "_" + str(object1.id), object2.name + "_" + str(object2.id))


# Creating ROS messages


def make_world_body(object: Object) -> 'WorldBody':
    """
    Creates a WorldBody message for a BulletWorld Object. The WorldBody will contain the URDF of the BulletWorld Object

    :param object: The BulletWorld Object
    :return: A WorldBody message for the BulletWorld Object
    """
    urdf_string = ""
    with open(object.path) as f:
        urdf_sting = f.read()
    urdf_body = WorldBody()
    urdf_body.type = WorldBody.URDF_BODY
    urdf_body.urdf = urdf_string

    return urdf_body


def make_point_stamped(point: List[float]) -> PointStamped:
    """
    Creates a PointStamped message for the given position in world coordinate frame.

    :param point: XYZ coordinates of the point
    :return: A PointStamped message
    """
    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.point.x = point[0]
    msg.point.y = point[1]
    msg.point.z = point[2]

    return msg


def make_quaternion_stamped(quaternion: List[float]) -> QuaternionStamped:
    """
    Creates a QuaternionStamped message for the given quaternion.

    :param quaternion: The quaternion as a list of xyzw
    :return: A QuaternionStamped message
    """
    msg = QuaternionStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.quaternion.x = quaternion[0]
    msg.quaternion.y = quaternion[1]
    msg.quaternion.z = quaternion[2]
    msg.quaternion.w = quaternion[3]

    return msg


def make_vector_stamped(vector: List[float]) -> Vector3Stamped:
    """
    Creates a Vector3Stamped message, this is similar to PointStamped but represents a vector instead of a point.

    :param vector: The vector given as xyz in world frame
    :return: A Vector3Stamped message
    """
    msg = Vector3Stamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.vector.x = vector[0]
    msg.vector.y = vector[1]
    msg.vector.z = vector[2]

    return msg


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


def move_head_to_human():
    """
    continously moves head in direction of perceived human
    """
    giskard_wrapper.continuous_pointing_head()
    giskard_wrapper.execute(wait=False, add_default=False)


def stop_looking():
    """
    stops the move_head_to_human function so that hsr looks forward
    """

    # cancels all goals in giskard
    # giskard_wrapper.cancel_all_goals()
    # moves hsr in standard position
    giskard_wrapper.take_pose("park")
    giskard_wrapper.execute(wait=False)
    rospy.loginfo("hsr looks forward instead of looking at human")


def cancel_all_called_goals():
    giskard_wrapper.cancel_all_goals()
    rospy.loginfo("Canceling all goals towards Giskard")


def move_head_to_pose(pose: PoseStamped):
    """
    moves head to given position
    :param pose: pose that head will rotate to
    """
    # TODO: necessary? there is a LookAtAction?
    # TODO: needs to be tested!
    p_axis = Vector3Stamped()
    p_axis.vector = (0, 0, 1)
    pointSt = PointStamped()
    pointSt.header = pose.header
    pointSt.point = pose.pose.position

    giskard_wrapper.set_pointing_goal(goal_point=_pose_to_pose_stamped(pointSt),
                                      tip_link="head_center_camera_frame",
                                      pointing_axis=p_axis,
                                      root_link="base_footprint")


def move_arm_to_pose(pose: PointStamped):
    """
    moves arm to given position
    :param pose: pose that arm will point to
    """
    # TODO: needs to be tested!
    print("in move arm")
    p_axis = Vector3Stamped()
    p_axis.header.frame_id = "hand_gripper_tool_frame"
    p_axis.vector.x = 0
    p_axis.vector.y = 0
    p_axis.vector.z = 1
    giskard_wrapper.set_pointing_goal(goal_point=pose,
                                      tip_link="hand_gripper_tool_frame",
                                      pointing_axis=p_axis,
                                      root_link="map")
    giskard_wrapper.execute()


def grasp_doorhandle(handle_name: str):
    print("grasp handle")

    giskard_wrapper.set_hsrb_door_handle_grasp(handle_name=handle_name)
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.execute()


def grasp_handle(handle_name: str):
    """
    grasps the dishwasher handle.

    :param handle_name: name of the dishwasher handle, which should be grasped
    """
    giskard_wrapper.set_hsrb_dishwasher_door_handle_grasp(handle_name, grasp_bar_offset=0.035)
    giskard_wrapper.execute()


def open_doorhandle(handle_name: str):
    giskard_wrapper.allow_all_collisions()
    giskard_wrapper.set_hsrb_open_door_goal(door_handle_link=handle_name)
    giskard_wrapper.execute(add_default=False)


def spawn_kitchen():
    env_urdf = rospy.get_param('/iai_kitchen')
    kitchen_pose = tf.lookup_pose('map', 'iai_kitchen/urdf_main')
    print(kitchen_pose)
    giskard_wrapper.add_urdf(name='arena',
                             urdf=env_urdf,
                             pose=kitchen_pose)


def place_objects(object, target, grasp):
    """
    function for placing objects
    :param object: ObjectDesignator describing the object
    :param target: target pose where to place the object
    :param grasp: how the object was picked up
    """

    from_above_objects = ["Bowl", "Metalmug", "Spoon", "Knife", "Fork"]

    context_from_above = {'action': 'placing', 'from_above': True}
    context_default = {'action': 'placing'}

    if object.name in from_above_objects:
        giskard_wrapper.placing(context=context_from_above, goal_pose=target)

    else:
        giskard_wrapper.placing(context="align_vertical", goal_pose=target)

    rospy.loginfo("placed object")


def park_arms():
    giskard_wrapper.take_pose("park")
    giskard_wrapper.execute()



# def reaching(self,
#                #context,
#                grasp: str -> front top right left below
#                align -> frame (dh wrist frame aligned damit) -> aka tip_link, wenn aliugn leer dnan ignore
#                object_name: str, #(die spawned planning)
#                object_shape: str, #(cylinder oder something lese)
#                goal_pose: Optional[PoseStamped] = None,
#                object_size: Optional[Vector3] = None,
#                root_link: str = 'map',
#                tip_link: str = 'hand_palm_link',
#                velocity: float = 0.2): -> auch von planning
