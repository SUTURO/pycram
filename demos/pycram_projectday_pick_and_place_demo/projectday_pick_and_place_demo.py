from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl, try_pick_up, get_free_spaces, \
    try_detect_with_tilting
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import real_robot
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

lt = LocalTransformer()

with_specific_objects = False

wished_sorted_obj_list = ["Milkpack"]

with_placing = True

# Initialize the Bullet world for simulation
world = BulletWorld(WorldMode.GUI)

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")


def sort_objects_demo(objs_list: List, wished_obj_list: List):
    tuples_list = []
    sorted_objects = []
    if len(objs_list) == 0:
        return sorted_objects

    for value in objs_list:
        object_type = value.obj_type
        if value.obj_type in ["Mueslibox", "Cornybox", "Cerealbox", "Crackerbox", "MuesliboxVitalis"]:
            object_type = "Cerealbox"
        if value.obj_type in ["Spoon", "Fork", "Knife", "Plasticknife"]:
            object_type = "Spoon"
        if value.obj_type in ["Milkpack", "Milkpackja", "MilkpackLactoseFree"]:
            object_type = "Milkpack"
        if object_type in wished_obj_list:
            tuples_list.append((value, wished_obj_list.index(object_type)))
    sorted_objects = [x[0] for x in sorted(tuples_list, key=lambda index: index[1])]

    # print which objects are in the final list
    test_list = []
    for test_object in sorted_objects:
        test_list.append(test_object.obj_type)
    print(test_list)

    return sorted_objects


with (real_robot):
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveTorsoAction([0.2]).resolve().perform()
    object_desig = try_detect_with_tilting(-0.4)
    print(object_desig)
    obj_list = []
    for value in object_desig.values():
        obj_list.append(value)

    if with_specific_objects:
        obj_list = sort_objects_demo(obj_list, wished_sorted_obj_list)

    object_pose = obj_list[0].pose
    grasp = Grasp.FRONT
    if obj_list[0].obj_type in ["Spoon", "Fork", "Knife", "Plasticknife"] or obj_list[0].obj_type == "Metalbowl":
        grasp = Grasp.TOP
        MoveTorsoAction([0.5]).resolve().perform()

    PickUpAction(obj_list[0], [Arms.LEFT], [grasp]).resolve().perform()

    rTm = robot.get_pose()
    rTb = lt.transform_pose(rTm, robot.get_link_tf_frame("base_link"))

    rTb.pose.position.x -= 0.45
    rTbm = lt.transform_pose(rTb, "map")
    NavigateAction(target_locations=[rTbm]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()

    if with_placing:
        PlaceAction(obj_list[0], [Pose([object_pose.position.x, object_pose.position.y, 0.713])],
                    [grasp], [Arms.LEFT], with_force_torque=[True]).resolve().perform()
        rTm = robot.get_pose()
        rTb = lt.transform_pose(rTm, robot.get_link_tf_frame("base_link"))

        rTb.pose.position.x -= 0.45
        rTbm = lt.transform_pose(rTb, "map")
        NavigateAction(target_locations=[rTbm]).resolve().perform()

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveTorsoAction([0.0]).resolve().perform()
