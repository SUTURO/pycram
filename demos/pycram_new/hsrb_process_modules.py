import numpy as np
import rospy
from threading import Lock
from typing import Any

from geometry_msgs.msg import PoseStamped, PointStamped

from ..datastructures.dataclasses import Color
from ..datastructures.enums import JointType, PerceptionTechniques
from ..external_interfaces.robokudo import query_human, faces_query, stop_query, query_specific_region, \
    query_human_attributes, send_query
from ..external_interfaces.tmc import tmc_gripper_control, tmc_talk
from ..robot_description import RobotDescription
from ..process_module import ProcessModule
from ..datastructures.pose import Point
from ..robot_descriptions import robot_description
from ..utils import _apply_ik
from ..external_interfaces.ik import request_ik
from .. import world_reasoning as btr
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..external_interfaces import giskard
from ..external_interfaces.navigate import PoseNavigator
from ..world_concepts.world_object import Object
from ..datastructures.world import World
from pycram.worlds.bullet_world import BulletWorld
from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription
from pydub import AudioSegment
from pydub.playback import play
from gtts import gTTS
import io


###########################################################
########## Process Modules for the Real HSRB ###############
###########################################################

class HSRBNavigationReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        move = PoseNavigator()
        rospy.loginfo(f"Sending goal to giskard to Move the robot")
        # giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        move.pub_now(designator.target)


class HSRBMoveHeadReal(ProcessModule):
    """
    Process module for the real HSRB that sends a pose goal to giskard to move the robot head
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        pose = PoseStamped()
        pose.pose.position.x = target.pose.position.x
        pose.pose.position.y = target.pose.position.y
        pose.pose.position.z = target.pose.position.z
        giskard.move_head_to_pose(pose)


class HSRBHeadFollowReal(ProcessModule):
    """
    HSR will move head to pose that is published on topic /human_pose
    """

    def _execute(self, designator: HeadFollowMotion) -> Any:
        if designator.state == 'stop':
            giskard.cancel_all_called_goals()
        else:
            giskard.move_head_to_human()


class HSRBPointingReal(ProcessModule):
    """
    HSR will move head to pose that is published on topic /human_pose
    """

    def _execute(self, designator: PointingMotion) -> Any:
        pointing_pose = PointStamped()
        pointing_pose.header.frame_id = "map"
        pointing_pose.point.x = designator.x_coordinate
        pointing_pose.point.y = designator.y_coordinate
        pointing_pose.point.z = designator.z_coordinate
        giskard.move_arm_to_point(pointing_pose)


class HSRBDetectingReal(ProcessModule):
    """
    Process Module for the real HSRB that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, desig: DetectingMotion) -> Any:
        # todo at the moment perception ignores searching for a specific object type so we do as well on real
        if desig.technique == 'human' and (desig.state == 'start' or desig.state == None):
            human_pose = query_human()
            return human_pose
        elif desig.state == "face":

            res = faces_query()
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

        elif desig.state == "stop":
            stop_query()
            return "stopped"

        elif desig.technique == 'location':
            seat = desig.state
            seat_human_pose = query_specific_region(seat)

            if seat == "long_table" or seat == "popcorn_table":
                loc_list = []
                for loc in seat_human_pose[0].attribute:
                    print(f"location: {loc}, type: {type(loc)}")
                    loc_list.append(loc)
                print(loc_list)
                return loc_list
                # return seat_human_pose[0].attribute
            # if only one seat is checked
            # TODO check if still needed
            if seat != "sofa":
                return seat_human_pose[0].attribute[0][9:].split(',')
            # when whole sofa gets checked, a list of lists is returned
            res = []
            for i in seat_human_pose[0].attribute:
                res.append(i.split(','))

            print(res)
            return res

        elif desig.technique == 'attributes':
            human_pose_attr = query_human_attributes()
            counter = 0
            # wait for human to come
            while not human_pose_attr.res and counter < 6:
                human_pose_attr = query_human_attributes()
                counter += 1
                if counter > 3:
                    TalkingMotion("please step in front of me").resolve().perform()
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

        # used when region-filter of robokudo should be used
        elif desig.technique == 'region':
            region = desig.state  # name of the region where should be perceived
            query_result = query_specific_region("region", region)
            perceived_objects = []

            for obj in query_result:
                # this has to be pose from pose stamped since we spawn the object with given header
                list = obj.pose
                if len(list) == 0:
                    continue
                obj_pose = Pose.from_pose_stamped(list[0])
                # obj_pose.orientation = [0, 0, 0, 1]
                # obj_pose_tmp = query_result.res[i].pose[0]
                obj_type = obj.type
                obj_size = obj.size
                # obj_color = query_result.res[i].color[0]

                # atm this is the string size that describes the object but it is not the shape size thats why string
                def extract_xyz_values(input_string):
                    # Split the input string by commas and colon to separate key-value pairs
                    # key_value_pairs = input_string.split(', ')

                    # Initialize variables to store the X, Y, and Z values
                    x_value = None
                    y_value = None
                    z_value = None

                    # todo: for now it is a string again, might be changed back. In this case we need the lower commented out code
                    xvalue = input_string[(input_string.find("x") + 2): input_string.find("y")]
                    y_value = input_string[(input_string.find("y") + 2): input_string.find("z")]
                    z_value = input_string[(input_string.find("z") + 2):]

                    # Iterate through the key-value pairs to extract the values
                    # for pair in key_value_pairs:
                    #     key, value = pair.split(': ')
                    #     if key == 'x':
                    #         x_value = float(value)
                    #     elif key == 'y':
                    #         y_value = float(value)
                    #     elif key == 'z':
                    #         z_value = float(value)

                    return x_value, y_value, z_value

                x, y, z = extract_xyz_values(obj_size)
                # size = (x, z / 2, y)
                # size_box = (x / 2, z / 2, y / 2)
                hard_size = (0.02, 0.02, 0.03)
                # TODO: add Bulletworld obj to fkt, replaced function add_ridig_box
                # gen_obj_desc = GenericObjectDescription("robokudo_object", [0, 0, 0], [0.1, 0.1, 0.1])
                # id = BulletWorld.load_generic_object_and_get_id(description=gen_obj_desc)
                # TODO: adjust right path, hardcoded for now
                # TODO: name is not unique anymore
                box_object = Object(obj_type, obj_type, pose=obj_pose, color=Color(0, 0, 0, 1), path="milk.stl")
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

                perceived_objects.append(box_desig)

            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(perceived_objects):
                object_dict[obj.name] = obj
            return object_dict

        else:
            query_result = send_query(ObjectDesignatorDescription(types=[ObjectType.MILK]))
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

                # if desig.object_type:
                #     if not desig.object_type.lower() in obj_type.lower():
                #         pass

                color_switch = {
                    "red": Color(1, 0, 0, 1),
                    "yellow": Color(1, 1, 0, 1),
                    "green": Color(0, 1, 0, 1),
                    "cyan": Color(0, 1, 1, 1),
                    "blue": Color(0, 0, 1, 1),
                    "magenta": Color(1, 0, 1, 1),
                    "white": Color(1, 1, 1, 1),
                    "black": Color(0, 0, 0, 1),
                    "grey": Color(0.5, 0.5, 0.5, 1),
                    # add more colors if needed
                }

                color = color_switch.get(obj_color)

                if color is None:
                    color = Color(0, 0, 0, 1)


                hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]
                osize = [obj_size.x, obj_size.y, obj_size.z]
                # TODO: add Bulletworld obj to fkt, replaced function add_ridig_box
                # gen_obj_desc = GenericObjectDescription("robokudo_object", [0, 0, 0], [0.1, 0.1, 0.1])
                # id = BulletWorld.load_generic_object_and_get_id(description=gen_obj_desc)

                # TODO: adjust right path, hardcoded for now
                # TODO: name not unique anymore
                box_object = Object(obj_type, obj_type, pose=obj_pose, color=color, path="milk.stl")
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, obj_type, box_object)

                perceived_objects.append(box_desig)

            # object_dict = {}
            #
            # for i, obj in enumerate(perceived_objects):
            #     print(obj.name)
            #     object_dict[obj.name] = obj
            return perceived_objects


class HSRBMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real HSRB while avoiding all collisions
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
    Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions
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
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBTalkReal(ProcessModule):
    """
    Tries to close an already grasped container
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
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.teleport_robot(designator.target)


class HSRBTalkSemiReal(ProcessModule):
    """
    Tries to close an already grasped container
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
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()
        self._talk_lock = Lock()
        self._head_follow_lock = Lock()
        self._pointing_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBNavigationReal(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBNavigationSemiReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveHeadReal(self._looking_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBDetectingReal(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveTCPReal(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveJointsReal(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveGripperReal(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBOpenReal(self._open_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBCloseReal(self._close_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBCloseReal(self._close_lock)

    def talk(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBTalkReal(self._talk_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBTalkSemiReal(self._talk_lock)

    def head_follow(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBHeadFollowReal(self._head_follow_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBHeadFollowReal(self._head_follow_lock)

    def pointing(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBPointingReal(self._pointing_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBPointingReal(self._pointing_lock)
