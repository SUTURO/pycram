from pycram.worlds.multiverse import Multiverse
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

extension = ObjectDescription.get_file_extension()

world = Multiverse()
robot = Object("tiago_dual", ObjectType.ROBOT, f"tiago_dual{extension}", pose=Pose([1, 2, 0]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment{extension}")

milk = Object("milk_box", ObjectType.MILK, f"milk_box{extension}", pose=Pose([2.5, 2, 1.02]),
              color=Color(1, 0, 0, 1))
spoon = Object("soup_spoon", ObjectType.SPOON, f"soup_spoon{extension}", pose=Pose([2.4, 2.2, 0.85]),
               color=Color(0, 0, 1, 1))
bowl = Object("big_bowl", ObjectType.BOWL, f"big_bowl{extension}", pose=Pose([2.5, 2.2, 1.02]),
              color=Color(1, 1, 0, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["tiago_dual"])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    milk_desig = move_and_detect(ObjectType.MILK)

    TransportAction(milk_desig, ["left"], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

    bowl_desig = move_and_detect(ObjectType.BOWL)

    TransportAction(bowl_desig, ["left"], [Pose([5, 3.3, 0.8], [0, 0, 1, 1])]).resolve().perform()

    # Finding and navigating to the drawer holding the spoon
    handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                        robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    spoon.detach(apartment)

    # Detect and pickup the spoon
    LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

    spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()

    pickup_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
    PickUpAction(spoon_desig, [pickup_arm], ["top"]).resolve().perform()

    ParkArmsAction([Arms.LEFT if pickup_arm == "left" else Arms.RIGHT]).resolve().perform()

    CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.15]).resolve().perform()

    # Find a pose to place the spoon, move and then place it
    spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
    placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()

    NavigateAction([placing_loc.pose]).resolve().perform()

    PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
