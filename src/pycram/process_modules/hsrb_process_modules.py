import numpy as np
from threading import Lock

import rospy
from typing_extensions import Any

from ..datastructures.dataclasses import Color
from ..datastructures.enums import ExecutionType, JointType
from ..external_interfaces.ik import request_ik
from ..external_interfaces.navigate import PoseNavigator
from ..external_interfaces.robokudo import *
from ..external_interfaces.tmc import tmc_gripper_control, tmc_talk
from ..robot_description import RobotDescription
from ..process_module import ProcessModule
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..external_interfaces import giskard
from ..datastructures.world import World
from pydub import AudioSegment
from pydub.playback import play
from gtts import gTTS

import io

from ..ros.logging import logdebug, loginfo, logerr
from ..utilities.robokudo_obj_translator import translate_obj
from ..utils import _apply_ik
from ..world_concepts.world_object import Object
from ..world_reasoning import link_pose_for_joint_config, visible

use_giskard = True

def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of HSRB and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = World.robot
    if arm == "left":
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_position(joint, pose)


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_tool_frame(arm)

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


class HSRBNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = World.robot
        robot.set_pose(desig.target)


class HSRBDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """


    def _execute(self, desig: DetectingMotion):
        loginfo("Detecting technique: {}".format(desig.technique))
        robot = World.robot
        object_type = desig.object_type
        cam_frame_name = RobotDescription.current_robot_description.get_camera_frame()
        front_facing_axis = RobotDescription.current_robot_description.get_default_camera().front_facing_axis
        if desig.technique == 'all':
            loginfo("Fake detecting all generic objects")
            objects = World.current_world.get_all_objects_not_robot()
        elif desig.technique == 'human':
            loginfo("Fake detecting human -> spawn 0,0,0")
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=Pose([0, 0, 0])))
            object_dict = {}

            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

        else:
            loginfo("Fake -> Detecting specific object type")
            objects = World.current_world.get_object_by_type(object_type)

        object_dict = {}

        perceived_objects = []
        for obj in objects:
            if visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                perceived_objects.append(ObjectDesignatorDescription.Object(obj.name, obj.obj_type, obj))

        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj

        loginfo("returning dict objects")
        return object_dict


class HSRBMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        if use_giskard:
            lt = LocalTransformer()
            pose_in_map = lt.transform_pose(designator.target, "map")
            giskard.avoid_all_collisions()
            if designator.allow_gripper_collision:
                giskard.allow_gripper_collision(designator.arm)
            giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
                designator.arm).get_tool_frame(), "map")
        else:
            _move_arm_tcp(designator.target, World.robot, designator.arm)


class HSRBMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, designator: MoveArmJointsMotion):
        if use_giskard:
            joint_goals = {}
            if designator.left_arm_poses:
                joint_goals.update(designator.left_arm_poses)
            giskard.avoid_all_collisions()
            giskard.achieve_joint_goal(joint_goals)
        else:
            robot = World.robot
            if designator.right_arm_poses:
                robot.set_multiple_joint_positions(designator.right_arm_poses)
            if designator.left_arm_poses:
                robot.set_multiple_joint_positions(designator.left_arm_poses)


class HSRBMoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, designator: MoveJointsMotion):
        if use_giskard:
            name_to_position = dict(zip(designator.names, designator.positions))
            giskard.avoid_all_collisions()
            giskard.achieve_joint_goal(name_to_position)
        else:
            robot = World.robot
            robot.set_multiple_joint_positions(dict(zip(designator.names, designator.positions)))


class HSRBWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, World.current_world.objects))[0]


class HSRBOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(container_joint)[1])


class HSRBClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(container_joint)[0])


class HSRBMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position("head_pan_joint")
        current_tilt = robot.get_joint_position("head_tilt_joint")

        robot.set_joint_position("head_pan_joint", new_pan + current_pan)
        robot.set_joint_position("head_tilt_joint", new_tilt + current_tilt)


class HSRBMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        robot = World.robot
        motion = desig.motion
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(
                desig.gripper).get_static_gripper_state(motion).items():
            robot.set_joint_position(joint, state)

###########################################################
########## Process Modules for the Real HSRB ###############
###########################################################


class HSRBNavigationReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        use_giskard = False
        if use_giskard:
            logdebug(f"Sending goal to giskard to Move the robot")
            giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")
        else:
            nav = PoseNavigator()
            nav.pub_now(designator.target)


class HSRBMoveHeadReal(ProcessModule):
    """
    Process module for the real HSRB that sends a pose goal to giskard to move the robot head
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, np.sqrt(pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2))

        current_pan = robot.get_joint_position("head_pan_joint")
        current_tilt = robot.get_joint_position("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(
            {"head_pan_joint": new_pan + current_pan, "head_tilt_joint": new_tilt + current_tilt})



class HSRBDetectingReal(ProcessModule):
    """
    Process Module for the real HSRB that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, desig: DetectingMotion) -> Any:
        """
        specifies the query send to robokudo
        :param desig.technique: if this is set to human the hsr searches for human and publishes the pose
        to /human_pose. returns PoseStamped of Human.
        this value can also be set to 'attributes', 'location' or 'region' to get the attributes and pose of a human, a bool
        if a seat specified in the sematic map is taken or to describe where objects should be perceived.

        """

        # ToDo: at the moment perception ignores searching for a specific object type so we do that as well on real
        if desig.state == "stop":
            stop_query()
            return "stopped"
        elif desig.technique == 'human' and (desig.state == 'start' or desig.state == None):
            human_pose = query_human()
            return human_pose
        elif desig.technique == 'drink':
            drinks = query_beverages()
            return drinks

        elif desig.technique == 'waving':
            query_result = query_waving_human()
            for i in range(0, len(query_result.res)):
                try:
                    human_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
                except IndexError:
                    human_pose = Pose.from_pose_stamped(query_result.res[i].pose)
                    pass

            return human_pose
        elif desig.state == "face":

            res = query_faces_human()
            id_dict = {}
            keys = []
            if res.res:
                for ele in res.res:
                    id_dict[int(ele.type)] = ele.pose[0]
                    keys.append(int(ele.type))
                id_dict["keys"] = keys
                return id_dict
            else:
                return []

        elif desig.technique == 'holding_drink':
            logerr("not implemented yet")
            return_list = []

            return return_list



        elif desig.technique == 'location':
            seat = desig.state
            seat_human_pose = query_specific_region(seat)

            if seat == "long_table" or seat == "popcorn_table":
                loc_list = []
                for loc in seat_human_pose[0].attribute:
                    loginfo(f"location: {loc}, type: {type(loc)}")
                    loc_list.append(loc)
                loginfo("".join(loc_list))
                return loc_list

            # if only one seat is checked
            if seat != "sofa":
                return seat_human_pose[0].attribute[0][9:].split(',')
            # when whole sofa gets checked, a list of lists is returned
            res = []

            for i in seat_human_pose.res[0].attribute:
                res.append(i.split(','))

            return res

        elif desig.technique == 'attributes':
            human_pose_attr = query_human_attributes()
            counter = 0
            # wait for human to come
            # TODO: try catch block
            while not human_pose_attr.res and counter < 6:
                human_pose_attr = query_human_attributes()
                counter += 1
                if counter > 3:
                    TalkingMotion("please step in front of me").perform()
                    rospy.sleep(2)

            if counter >= 3:
                return "False"

            # extract information from query
            gender = human_pose_attr.res[0].attribute[3][13:19]
            if gender[0] != 'f':
                gender = gender[:4]
            clothes = human_pose_attr.res[0].attribute[1][20:]
            brightness_clothes = human_pose_attr.res[0].attribute[0][5:]
            hat = human_pose_attr.res[0].attribute[2][20:]
            attr_list = [gender, hat, clothes, brightness_clothes]
            return attr_list

        # detect objects in a certain area
        elif desig.technique == "region":
            query_result = query_specific_region()
            perceived_objects = []
            if query_result:
                for key,obj in query_result.items():
                    # this has to be pose from pose stamped since we spawn the object with given header
                    list = obj.pose
                    if len(list) == 0:
                        continue
                    obj_pose = Pose.from_pose_stamped(list[0])
                    obj_type = obj.type
                    obj_size = obj.size

                    # atm this is the string size that describes the object, but it is not the shape size thats why string
                    def extract_xyz_values(input_string):
                        # Initialize variables to store the X, Y, and Z values
                        x_value = None
                        y_value = None
                        z_value = None

                        # todo: for now it is a string again, might be changed back. In this case we need the lower commented out code
                        xvalue = input_string[(input_string.find("x") + 2): input_string.find("y")]
                        y_value = input_string[(input_string.find("y") + 2): input_string.find("z")]
                        z_value = input_string[(input_string.find("z") + 2):]

                        return x_value, y_value, z_value

                    hard_size = (0.02, 0.02, 0.03)
                    id = World.current_world.add_rigid_box(obj_pose, hard_size, [0, 0, 0, 1])
                    box_object = Object(obj_type + "" + str(rospy.get_time()), obj_type, pose=obj_pose,
                                        color=Color(0, 0, 0, 1),
                                        custom_id=id,
                                        custom_geom={"size": [hard_size[0], hard_size[1], hard_size[2]]})
                    box_object.set_pose(obj_pose)
                    box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

                    perceived_objects.append(box_desig)

                object_dict = {}

                # Iterate over the list of objects and store each one in the dictionary
                for i, obj in enumerate(perceived_objects):
                    object_dict[obj.name] = obj
                return object_dict

        # technique == "all", detects all objects
        else:
            query_result = send_query()
            perceived_objects = []
            for i in range(0, len(query_result.res)):
                try:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
                except IndexError:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose)
                    pass
                obj_type = query_result.res[i].type
                obj_size = None
                try:
                    obj_size = query_result.res[i].shape_size[0].dimensions
                except IndexError:
                    pass
                obj_color = None
                try:
                    obj_color = query_result.res[i].color[0]
                except IndexError:
                    pass

                color_switch = {
                    "red": [1, 0, 0, 1],
                    "yellow": [1, 1, 0, 1],
                    "green": [0, 1, 0, 1],
                    "cyan": [0, 1, 1, 1],
                    "blue": [0, 0, 1, 1],
                    "magenta": [1, 0, 1, 1],
                    "white": [1, 1, 1, 1],
                    "black": [0, 0, 0, 1],
                    "grey": [0.5, 0.5, 0.5, 1],
                    # add more colors if needed
                }

                color = color_switch.get(obj_color)
                if color is None:
                    color = Color(0, 0, 0, 1)

                hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]
                osize = [obj_size.x, obj_size.y, obj_size.z]
                id = World.current_world.add_rigid_box(obj_pose, hsize, color)
                path = translate_obj(obj_type)
                box_object = Object(obj_type + "_" + str(rospy.get_time()), obj_type, pose=obj_pose, color=color,
                                    custom_id=id,
                                    custom_geom={"size": osize}, path=path)
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.obj_type, box_object)

                perceived_objects.append(box_desig)

            object_dict = {}

            for i, obj in enumerate(perceived_objects):
                object_dict[obj.name] = obj
            return object_dict


class HSRBMoveTCPForceTorqueReal(ProcessModule):
    """
    Moves the tool center point of the real HSRB while avoiding all collisions via giskard with force torque data
    """

    def _execute(self, designator: MoveTCPForceTorqueMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal_w_fts(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
            designator.arm).get_tool_frame(), 'map', designator.object_type, designator.threshold)
        # giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
        #     designator.arm).get_tool_frame(), "map")


class HSRBMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real HSRB while avoiding all collisions via giskard
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
            designator.arm).get_tool_frame(), "map")


class HSRBMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions via giskard
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class HSRBMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class HSRBMoveGripperReal(ProcessModule):
    """
     Opens or closes the gripper of the real HSRB with the help of giskard.
     """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        tmc_gripper_control(designator)


class HSRBOpenReal(ProcessModule):
    """
    This process Modules tries to open an already grasped container via giskard
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBCloseReal(ProcessModule):
    """
    This process module executes close a an already grasped container via giskard
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBTalkReal(ProcessModule):
    """
    Let the robot speak over tmc interface.
    """

    def _execute(self, designator: TalkingMotion) -> Any:
        tmc_talk(designator)


###########################################################
########## Process Modules for the Semi Real HSRB ###############
###########################################################
class HSRBNavigationSemiReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        logdebug(f"Sending goal to giskard to Move the robot")
        giskard.teleport_robot(designator.target)


class HSRBTalkSemiReal(ProcessModule):
    """
    Low Level implementation to let the robot talk using gTTS and pydub.
    """

    def _execute(self, designator: TalkingMotion) -> Any:
        """
        Convert text to speech using gTTS, modify the pitch and play it without saving to disk.
        """
        sentence = designator.cmd
        # Create a gTTS object
        tts = gTTS(text=sentence, lang='en', slow=False)

        # Save the speech to an in-memory file
        mp3_fp = io.BytesIO()
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)

        # Load the audio into pydub from the in-memory file
        audio = AudioSegment.from_file(mp3_fp, format="mp3")

        # Speed up the audio slightly
        faster_audio = audio.speedup(playback_speed=1.2)

        # Play the modified audio
        play(faster_audio)

class HSRBPourReal(ProcessModule):
    """
    Tries to achieve the pouring motion
    """

    def _execute(self, designator: PouringMotion) -> Any:
        giskard.achieve_tilting_goal(designator.direction, designator.angle)


class HSRBHeadFollowReal(ProcessModule):
    """
    HSR will move head to pose that is published on topic /human_pose
    """

    def _execute(self, designator: HeadFollowMotion) -> Any:
        if designator.state == 'stop':
            giskard.cancel_goal()
        else:
            giskard.move_head_to_human()


class HSRBPointingReal(ProcessModule):
    """
    HSR will move head to pose that is published on topic /human_pose
    """

    def _execute(self, designator: PointingMotion) -> Any:
        pointing_pose = designator.goal_point
        giskard.move_arm_to_point(pointing_pose)


class HSRBOpenDoorReal(ProcessModule):
    """
    HSR will perform open action on grasped handel for a door
    """

    def _execute(self, designator: DoorOpenMotion) -> Any:
        giskard.open_doorhandle(designator.handle)


class HSRBGraspHandleReal(ProcessModule):
    """
    HSR will grasp given (door-)handle
    """

    def _execute(self, designator: GraspHandleMotion) -> Any:
        giskard.grasp_doorhandle(designator.handle, designator.offset)


class HSRBGraspDishwasherHandleReal(ProcessModule):
    """Grasps the dishwasher handle"""

    def _execute(self, designator: GraspingDishwasherHandleMotion) -> Any:
        giskard.grasp_handle(designator.handle_name)


class HSRBHalfOpenDishwasherReal(ProcessModule):
    """Partially opens the dishwasher door."""

    def _execute(self, designator: HalfOpeningDishwasherMotion) -> Any:
        giskard.achieve_open_container_goal(RobotDescription.current_robot_description.get_arm_chain(designator.arm)
                                            .get_tool_frame(), designator.handle_name,
                                            goal_state=designator.goal_state_half_open, special_door=False)


class HSRBMoveArmAroundDishwasherReal(ProcessModule):
    """Moves the HSR arm around the dishwasher door after partially opening"""

    def _execute(self, designator: MoveArmAroundMotion) -> Any:
        giskard.set_hsrb_dishwasher_door_around(designator.handle_name)


class HSRBFullOpenDishwasherReal(ProcessModule):
    """Opens the dishwasher fully"""

    def _execute(self, designator: FullOpeningDishwasherMotion) -> Any:
        giskard.fully_open_dishwasher_door(designator.handle_name, designator.door_name)
        giskard.achieve_open_container_goal(RobotDescription.current_robot_description.get_arm_chain(designator.arm)
                                            .get_tool_frame(), designator.handle_name,
                                            goal_state=designator.goal_state_full_open, special_door=True)


###########################################################
########## HSRB MANAGER ###############
###########################################################
class HSRBManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("hsrb")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_tcp_ft_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._grasp_dishwasher_lock = Lock()
        self._move_around_lock = Lock()
        self._half_open_lock = Lock()
        self._full_open_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()
        self._talk_lock = Lock()
        self._pour_lock = Lock()
        self._head_follow_lock = Lock()
        self._pointing_lock = Lock()
        self._open_door_lock = Lock()
        self._grasp_handle_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBNavigationReal(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBNavigationSemiReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveHeadReal(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBDetectingReal(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveTCPReal(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveTCPReal(self._move_tcp_lock)

    def move_tcp_ft(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBMoveTCPForceTorqueReal(self._move_tcp_ft_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveTCPForceTorqueReal(self._move_tcp_ft_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveTCPForceTorqueReal(self._move_tcp_ft_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED or ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveJointsReal(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveGripperReal(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveGripperReal(self._move_gripper_lock)

    def grasp_dishwasher_handle(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBGraspDishwasherHandleReal(self._grasp_dishwasher_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBGraspDishwasherHandleReal(self._grasp_dishwasher_lock)

    def half_open_dishwasher(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBHalfOpenDishwasherReal(self._half_open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBHalfOpenDishwasherReal(self._half_open_lock)

    def move_arm_around_dishwasher(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveArmAroundDishwasherReal(self._move_around_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveArmAroundDishwasherReal(self._move_around_lock)

    def full_open_dishwasher(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBFullOpenDishwasherReal(self._full_open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBFullOpenDishwasherReal(self._full_open_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBOpenReal(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBCloseReal(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBCloseReal(self._close_lock)

    def talk(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBTalkReal(self._talk_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBTalkReal(self._talk_lock)

    def pour(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBPourReal(self._pour_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBPourReal(self._pour_lock)

    def head_follow(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBHeadFollowReal(self._head_follow_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBHeadFollowReal(self._head_follow_lock)

    def pointing(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBPointingReal(self._pointing_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBPointingReal(self._pointing_lock)

    def door_opening(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBOpenDoorReal(self._open_door_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBOpenDoorReal(self._open_door_lock)

    def grasp_door_handle(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBGraspHandleReal(self._grasp_handle_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBGraspHandleReal(self._grasp_handle_lock)
